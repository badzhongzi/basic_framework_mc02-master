#include "arm_mpc_task.h"

#include <math.h>
#include <string.h>

#include "bsp_dwt.h"
#include "mpc_6axis.h"
#include "rnea_6axis.h"

/*
 * FOLLOW 下目标交给轨迹层的最短间隔。
 *
 * 每次交给轨迹层都要重新规划一段五次多项式并做可行性检查, 目标每 2 ms
 * 变一次时逐周期重规划纯属浪费。10 ms 最多交一次, 每段平滑 100 ms, 相邻
 * 两段充分重叠, 参考值是连续的; 间隔内的目标变化不会丢, 下一次交的就是
 * 最新的目标。TIMED 的目标一变就交, 不受这个间隔限制。
 */
#define ARM_MPC_FOLLOW_PERIOD_TICK 10u

/*
 * 跟踪误差保护。|q_ref - q| 超过阈值且持续一段时间就判故障。
 * 方向配反、零位偏置配错、力矩方向反了时, 机械臂会越跟越远而不是
 * 抖一下, 这个保护就是为这类上电第一次必然会遇到的问题准备的。
 * 阈值是占位值, 调好以后可以收紧。
 */
#define ARM_MPC_TRACK_ERR_MAX 0.35f /* rad, 约 20 deg */
#define ARM_MPC_TRACK_ERR_TICKS (200u / INS_ARM_CONTROL_TICK_MS) /* 200 ms */

/* OSQP 状态码: 1 = solved, 2 = solved inaccurate。 */
#define ARM_MPC_OSQP_SOLVED            1
#define ARM_MPC_OSQP_SOLVED_INACCURATE 2

/* 达妙反馈状态 >= 8 为电机报错, 见 DMMotor_State_e。 */
#define ARM_MPC_DM_ERR_MIN DM_ERR_OVER_VOLTAGE

static float clampf(float v, float lim) {
  if (v > lim) return lim;
  if (v < -lim) return -lim;
  return v;
}

static bool joint_config_valid(const ArmMpcJointConfig *c) {
  return (c->direction == 1.0f || c->direction == -1.0f) &&
         isfinite(c->q_offset) && isfinite(c->tau_max) && c->tau_max > 0.0f;
}

/* ---------- 反馈 ---------- */

/*
 * 读取已接电机的关节角/角速度, 返回本周期检测到的故障位, 同时刷新
 * motor_online。
 *
 * 达妙电机的位置是输出端绝对位置 (零位由上位机预先设置), 角速度直接用
 * 电机回传值, 不做角度差分。
 *
 * 失能也按掉线处理: 电机晚于主控上电、掉电重启后都是失能状态, 驱动会自动
 * 补发使能, 在那之前它不会执行力矩指令, 算出来的前馈是空转的。
 */
static uint8_t read_joint_feedback(ArmMpcTask *self) {
  uint8_t flags = 0u;
  uint8_t online = 0u;
  uint8_t j;

  for (j = 0; j < ARM_MPC_JOINT_COUNT; ++j) {
    const DMMotorInstance *m = self->motors[j];
    const ArmMpcJointConfig *c = &self->joint_cfg[j];
    float q;
    float dq;

    if (!m) continue;
    /* feedback_cnt 在第一帧反馈到达时才会增加; daemon 在注册后前 100 ms 内
     * 即使一帧没收到也报在线, 只看 daemon 会把"从没收到过"当成在线。 */
    if (m->measure.feedback_cnt == 0u || !DaemonIsOnline(m->motor_daemon)) {
      flags |= ARM_MPC_FAULT_MOTOR_OFFLINE;
      continue;
    }
    if (m->measure.state >= (uint8_t)ARM_MPC_DM_ERR_MIN) {
      flags |= ARM_MPC_FAULT_MOTOR_ERROR;
      continue;
    }
    if (m->measure.state != (uint8_t)DM_STATE_ENABLED) {
      flags |= ARM_MPC_FAULT_MOTOR_OFFLINE;
      continue;
    }
    q = c->direction * m->measure.position + c->q_offset;
    dq = c->direction * m->measure.velocity;
    if (!isfinite(q) || !isfinite(dq)) {
      flags |= ARM_MPC_FAULT_FEEDBACK;
      continue;
    }
    self->q_actual[j] = q;
    self->dq_actual[j] = dq;
    online |= (uint8_t)(1u << j);
  }
  self->motor_online = online;
  return flags;
}

/* 未接电机的关节按理想跟踪处理: 反馈就是参考。这样可以一个关节一个关节地
 * 上电调试, 其余关节仍以参考构型参与 RNEA, 前馈的耦合项是对的。 */
static void fill_unconnected_joints(ArmMpcTask *self) {
  uint8_t j;
  for (j = 0; j < ARM_MPC_JOINT_COUNT; ++j) {
    if (self->motors[j]) continue;
    self->q_actual[j] = self->ref.q[j];
    self->dq_actual[j] = self->ref.q_dot[j];
  }
}

/* 故障确认: 对报错的电机请求清错。驱动按 DM_MODE_CMD_INTERVAL 限频发送,
 * 清错后电机回到失能状态, 再由驱动自动重新使能。 */
static void clear_motor_errors(ArmMpcTask *self) {
  uint8_t j;
  for (j = 0; j < ARM_MPC_JOINT_COUNT; ++j) {
    DMMotorInstance *m = self->motors[j];
    if (m && m->measure.state >= (uint8_t)ARM_MPC_DM_ERR_MIN) {
      DMMotorClearError(m);
    }
  }
}

/* ---------- 目标角 ---------- */

/* 零力矩期间目标角跟着实际角走, 恢复出力时就停在原地。 */
static void target_follow_actual(ArmMpcTask *self) {
  memcpy(self->q_target, self->q_actual, sizeof(self->q_target));
  memcpy(self->q_goal, self->q_actual, sizeof(self->q_goal));
  self->goal_result = (uint8_t)EC_TRAJ_OK;
  self->arrived = false;
}

/*
 * 目标角变了就交给轨迹层。两种方式都走绝对目标接口, 区别只在时长:
 * FOLLOW 用固定平滑时长, TIMED 用上层给的 move_time。目标已经在
 * ArmMpcTaskSetTarget() 里夹进限位, 所以不会因越限被拒绝。
 */
static void dispatch_target(ArmMpcTask *self) {
  const bool follow = self->move == (uint8_t)ARM_MPC_MOVE_FOLLOW;
  uint8_t j;

  if (memcmp(self->q_target, self->q_goal, sizeof(self->q_goal)) == 0) return;
  if (follow &&
      (uint32_t)(self->tick - self->goal_tick) < ARM_MPC_FOLLOW_PERIOD_TICK) {
    return;
  }

  self->goal_result = (uint8_t)ins_traj_command_absolute(
      &self->traj, self->tick, self->q_target,
      follow ? self->traj.jog_smooth_time : self->move_time,
      follow ? INS_TRAJ_SOURCE_MANUAL : INS_TRAJ_SOURCE_SEQUENCE);
  /* 失败也记下这个目标, 不逐周期重试; 目标再变一次才会重新规划。
   * 失败期间 arrived 保持 false, 上层的自动流程会停在这一步。 */
  for (j = 0; j < ARM_MPC_JOINT_COUNT; ++j) self->q_goal[j] = self->q_target[j];
  self->goal_tick = self->tick;
}

/* 参考值已经停在目标角: 目标全部交出去了、规划成功、轨迹走完。 */
static bool target_arrived(const ArmMpcTask *self) {
  return self->goal_result == (uint8_t)EC_TRAJ_OK &&
         memcmp(self->q_target, self->q_goal, sizeof(self->q_goal)) == 0 &&
         ins_traj_status(&self->traj) == (uint8_t)EC_TRAJ_HOLD;
}

/* ---------- RNEA 前馈 ---------- */

/* 前馈用实际 q/dq + 参考 ddq: 前馈要抵消的是真实动力学, 而 ddq 是
 * "希望关节产生的加速度"。传参考 q/dq 进去就变成纯开环前馈。 */
static void compute_rnea_feedforward(ArmMpcTask *self) {
  const casadi_real *arg[rnea_SZ_ARG] = {self->q_actual, self->dq_actual,
                                         self->ref.q_ddot};
  casadi_real *res[rnea_SZ_RES] = {self->tau_ff};

  /* rnea_SZ_IW = rnea_SZ_W = 0, 不需要工作内存。 */
  (void)rnea(arg, res, NULL, NULL, 0);
}

/* ---------- MPC 错峰求解 ---------- */

/*
 * delta_seq[k] = x_ref(t0) - x_ref(t0 + k * MPC_DT), k = 0..N。
 * t0 就是本周期的 tick, 和 self->ref 是同一个时刻, 所以 delta[0] 恒为 0,
 * 直接写 0 而不是求值后相减, 省一次求值也避免浮点上的非零残差。
 */
static void build_delta_seq(const EC_TrajPlan *plan, uint32_t tick,
                            uint8_t joint, float q0, float dq0,
                            float delta_seq[MPC_N + 1][2]) {
  const float t0 = (float)(int32_t)(tick - plan->t0_tick) * plan->tick_period;
  int k;

  delta_seq[0][0] = 0.0f;
  delta_seq[0][1] = 0.0f;
  for (k = 1; k <= MPC_N; ++k) {
    float qk = q0;
    float dqk = dq0;
    (void)ec_traj_eval_joint_time(plan, t0 + (float)k * MPC_DT, joint, &qk,
                                  &dqk, NULL);
    delta_seq[k][0] = q0 - qk;
    delta_seq[k][1] = dq0 - dqk;
  }
}

/*
 * 每周期解 MPC_JOINTS_PER_TICK 个关节, 其余关节沿用上次的 dtau。
 * 当前生成参数: 2 ms 周期 × 每次 2 个关节, 6 个关节轮一圈 6 ms = MPC_DT。
 * 这个对应关系由生成器保证, 这里用静态断言防止两边参数漂开。
 */
_Static_assert(MPC_NJ == ARM_MPC_JOINT_COUNT, "MPC joint count mismatch");
_Static_assert(MPC_NJ % MPC_JOINTS_PER_TICK == 0,
               "MPC_JOINTS_PER_TICK must divide MPC_NJ");

static void mpc_staggered_solve(ArmMpcTask *self, const EC_TrajPlan *plan) {
  float anchor[2];
  float delta_seq[MPC_N + 1][2];
  float q_vec[MPC_NZ], l_vec[MPC_NC], u_vec[MPC_NC];
  uint8_t i;

  for (i = 0; i < MPC_JOINTS_PER_TICK; ++i) {
    const uint8_t j = self->mpc_joint_index;
    int status;

    self->mpc_joint_index = (uint8_t)((j + 1u) % ARM_MPC_JOINT_COUNT);

    anchor[0] = self->ref.q[j] - self->q_actual[j];
    anchor[1] = self->ref.q_dot[j] - self->dq_actual[j];
    build_delta_seq(plan, self->tick, j, self->ref.q[j], self->ref.q_dot[j],
                    delta_seq);

    /* 钳位在 mpc_build_vectors 内部完成。 */
    mpc_build_vectors((int)j, anchor, delta_seq, q_vec, l_vec, u_vec);

    /* update/solve 调用失败 (参数或内部错误) 时沿用上次的 dtau, 不写 0:
     * 突然撤掉修正量比用一个过期值更糟。 */
    if (mpc_update_joint((int)j, q_vec, l_vec, u_vec) != 0 ||
        mpc_solve_joint((int)j) != 0) {
      ++self->mpc_fail_count;
      continue;
    }

    /* 未收敛 (max_iter 用尽) 时也采用当前迭代点, 和仿真的
     * USE_UNCONVERGED_ITERATE=True 保持一致; 只计数, 不丢弃。 */
    status = mpc_get_status((int)j);
    if (status != ARM_MPC_OSQP_SOLVED &&
        status != ARM_MPC_OSQP_SOLVED_INACCURATE) {
      ++self->mpc_unconverged_count;
    }
    self->mpc_dtau[j] = mpc_get_first_dtau((int)j); /* 已钳到 ±MPC_DTAU_MAX */
  }
}


/* ---------- 输出 ---------- */

static void output_zero_force(ArmMpcTask *self) {
  uint8_t j;

  memset(self->tau_ff, 0, sizeof(self->tau_ff));
  memset(self->mpc_dtau, 0, sizeof(self->mpc_dtau));
  memset(self->tau_total, 0, sizeof(self->tau_total));
  for (j = 0; j < ARM_MPC_JOINT_COUNT; ++j) {
    if (!self->motors[j]) continue;
    /* DMMotorStop 让 MIT 帧的力矩直接置零 (Kp=Kd=0), 电机保持使能以便继续
     * 收反馈; 同时把设定值清零, 免得重新出力那一刻先发出一个旧值。 */
    DMMotorSetRef(self->motors[j], 0.0f);
    DMMotorStop(self->motors[j]);
  }
}

/*
 * 总力矩 = RNEA 前馈 + MPC 修正, 按关节力矩上限饱和后直接以 N·m 下发。
 * MPC 只管住了 dtau, tau_ff 里的科氏/离心项按 dq^2 增长, 总力矩本身必须
 * 在这里兜底。tau_max 在初始化时已经和电机的 T_MAX 取过小值。
 * 返回 false 表示结果非有限值 (不应发生, 发生就按故障处理)。
 */
static bool output_torque(ArmMpcTask *self) {
  uint8_t j;

  for (j = 0; j < ARM_MPC_JOINT_COUNT; ++j) {
    const ArmMpcJointConfig *c = &self->joint_cfg[j];
    const float tau_raw = self->tau_ff[j] + self->mpc_dtau[j];

    if (!isfinite(tau_raw)) return false;
    self->tau_total[j] = clampf(tau_raw, c->tau_max);
    if (self->tau_total[j] != tau_raw) ++self->saturate_count;
  }

  for (j = 0; j < ARM_MPC_JOINT_COUNT; ++j) {
    if (!self->motors[j]) continue;
    DMMotorSetRef(self->motors[j],
                  self->joint_cfg[j].direction * self->tau_total[j]);
    DMMotorEnable(self->motors[j]);
  }
  return true;
}

/* 跟踪误差持续超限则返回故障位。 */
static uint8_t check_tracking(ArmMpcTask *self) {
  uint8_t j;
  bool over = false;

  for (j = 0; j < ARM_MPC_JOINT_COUNT; ++j) {
    if (!self->motors[j]) continue;
    if (fabsf(self->ref.q[j] - self->q_actual[j]) > ARM_MPC_TRACK_ERR_MAX) {
      over = true;
    }
  }
  if (!over) {
    self->track_err_ticks = 0u;
    return 0u;
  }
  if (self->track_err_ticks < ARM_MPC_TRACK_ERR_TICKS) ++self->track_err_ticks;
  return self->track_err_ticks >= ARM_MPC_TRACK_ERR_TICKS
             ? ARM_MPC_FAULT_TRACKING
             : 0u;
}

/*
 * 进入零力矩。参考值冻结在当前参考点 (轨迹层进入 FAULT, 拒绝一切新命令),
 * 电机不出力。离开时由 leave_limp() 以实际反馈角为新的保持点, 所以零力矩
 * 期间机械臂被重力拉走或被人推动都不会在恢复时往回跳。
 */
static void enter_limp(ArmMpcTask *self) {
  if (!self->limp) {
    ins_traj_fault(&self->traj, self->tick);
    self->limp = true;
  }
  self->track_err_ticks = 0u;
  output_zero_force(self);
}

static void leave_limp(ArmMpcTask *self) {
  /* 未接的关节此刻的 q_actual 就是冻结时的参考值, 已接的关节都是有效反馈
   * (有任何反馈故障都走不到这里)。 */
  ins_traj_clear_fault(&self->traj, self->tick, self->q_actual);
  target_follow_actual(self); /* 目标就是保持点, 恢复出力后原地不动 */
  memset(self->mpc_dtau, 0, sizeof(self->mpc_dtau));
  self->limp = false;
}

/* ========== 公共接口 ========== */

bool ArmMpcTaskInit(ArmMpcTask *self,
                    DMMotorInstance *motors[ARM_MPC_JOINT_COUNT],
                    const ArmMpcJointConfig joint_cfg[ARM_MPC_JOINT_COUNT]) {
  /* 此时还没有电机反馈, 先停在零位并立即进入零力矩; 真正的保持点在第一次
   * 离开零力矩时取实际反馈角 (见 leave_limp)。 */
  const float q_init[ARM_MPC_JOINT_COUNT] = {0};
  const float dq_max[ARM_MPC_JOINT_COUNT] = {2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f};
  const float ddq_max[ARM_MPC_JOINT_COUNT] = {20.0f, 20.0f, 20.0f,
                                              20.0f, 20.0f, 20.0f};
  uint8_t j;

  if (!self || !joint_cfg) return false;
  memset(self, 0, sizeof(*self));

  for (j = 0; j < ARM_MPC_JOINT_COUNT; ++j) {
    if (!joint_config_valid(&joint_cfg[j])) return false;
    self->joint_cfg[j] = joint_cfg[j];
    self->motors[j] = motors ? motors[j] : NULL;
    /* 超过电机 MIT 映射范围的力矩发不出去, 饱和计数要按真正发出去的值算。 */
    if (self->motors[j] &&
        self->joint_cfg[j].tau_max > self->motors[j]->limit.t_max) {
      self->joint_cfg[j].tau_max = self->motors[j]->limit.t_max;
    }
  }
  self->mode = (uint8_t)ARM_MPC_MODE_ZERO_FORCE;
  self->move = (uint8_t)ARM_MPC_MOVE_FOLLOW;
  self->move_time = 1.0f;
  self->tick = (uint32_t)osKernelSysTick();

  if (!ins_arm_init(&self->arm)) return false;
  if (!ins_traj_init(&self->traj, &self->arm, q_init, self->tick)) return false;
  if (!ins_traj_set_velocity_limit(&self->traj, dq_max, ddq_max)) return false;
  if (!ins_traj_set_jog_time(&self->traj, INS_TRAJ_DEFAULT_JOG_TIME)) {
    return false;
  }
  ins_traj_set_timeout(&self->traj, INS_TRAJ_DEFAULT_TIMEOUT_TICK);
  if (!ec_traj_eval_tick(ins_traj_plan(&self->traj), self->tick, &self->ref)) {
    return false;
  }

  self->initialized = true;
  enter_limp(self);
  return true;
}

void ArmMpcTaskSetMode(ArmMpcTask *self, ArmMpcMode mode) {
  if (!self) return;
  if (mode != ARM_MPC_MODE_ZERO_FORCE && mode != ARM_MPC_MODE_RUN) {
    mode = ARM_MPC_MODE_ZERO_FORCE;
  }
  self->mode = (uint8_t)mode;
}

void ArmMpcTaskSetTarget(ArmMpcTask *self, ArmMpcMove move,
                         const float *q_target, float move_time) {
  const EC_TrajLimits *lim;
  uint8_t j;

  if (!self || !self->initialized || !q_target) return;
  /* 零力矩/故障期间目标由 Run 改写成实际角, 这里写了也会被覆盖。 */
  if (self->limp) return;
  for (j = 0; j < ARM_MPC_JOINT_COUNT; ++j) {
    if (!isfinite(q_target[j])) return;
  }

  lim = &self->traj.limits;
  for (j = 0; j < ARM_MPC_JOINT_COUNT; ++j) {
    float q = q_target[j];
    if (q < lim->q_min[j]) q = lim->q_min[j];
    if (q > lim->q_max[j]) q = lim->q_max[j];
    self->q_target[j] = q;
  }
  self->move = (move == ARM_MPC_MOVE_TIMED) ? (uint8_t)ARM_MPC_MOVE_TIMED
                                            : (uint8_t)ARM_MPC_MOVE_FOLLOW;
  if (isfinite(move_time)) {
    if (move_time < lim->min_duration) move_time = lim->min_duration;
    if (move_time > lim->max_duration) move_time = lim->max_duration;
    self->move_time = move_time;
  }
}

InsTraj *ArmMpcTaskTraj(ArmMpcTask *self) {
  return self ? &self->traj : NULL;
}

/*
 * 每个控制周期调用一次。降级规则:
 *
 *   条件                                  处置
 *   ------------------------------------  ----------------------------------
 *   模式 ZERO_FORCE                       零力矩; 故障位每周期重新检测, 不锁存;
 *                                         报错的电机发清错
 *   电机掉线/从未上线/失能, 反馈非有限值  锁存故障 → 零力矩
 *   电机自身报错                          锁存故障 → 零力矩
 *   轨迹求值失败, 力矩非有限值            锁存故障 → 零力矩
 *   跟踪误差持续超限                      锁存故障 → 零力矩
 *   零力矩/故障期间                       目标角跟随实际角, 设置的目标不起作用
 *
 * 锁存故障只能通过切回 ZERO_FORCE 清除, 这是刻意的: 电机掉线又自己恢复时,
 * 不应该在没人确认的情况下突然重新出力。从零力矩恢复时, 保持点取实际反馈角。
 * 零力矩 = 电机不出力, 带重力负载的关节会下落, 上层要保证只在安全姿态或有人
 * 看护时进入, 需要"保持不动"的情况应该让目标角停住而不是零力矩。
 */
void ArmMpcTaskRun(ArmMpcTask *self) {
  const EC_TrajPlan *plan;
  EC_Arm *arm_state;
  uint32_t cycle_start;
  uint8_t detected;

  if (!self || !self->initialized) return;
  self->tick = (uint32_t)osKernelSysTick();
  arm_state = ins_arm_state(&self->arm);

  /* ---- 1. 反馈与故障检测 ---- */
  detected = read_joint_feedback(self);
  if (self->mode == (uint8_t)ARM_MPC_MODE_ZERO_FORCE) {
    self->fault_flags = detected;
    if (detected & ARM_MPC_FAULT_MOTOR_ERROR) clear_motor_errors(self);
  } else {
    self->fault_flags |= detected;
  }

  if (self->mode == (uint8_t)ARM_MPC_MODE_ZERO_FORCE || self->fault_flags) {
    /* 参考值仍然求值, 未接关节的"反馈"要靠它, 调试时也能看到冻结点。 */
    (void)ins_traj_run(&self->traj, arm_state, self->tick, &self->ref);
    fill_unconnected_joints(self);
    enter_limp(self);
    target_follow_actual(self);
    return;
  }
  if (self->limp) leave_limp(self);

  /* ---- 2. 目标角 ---- */
  dispatch_target(self);

  /* ---- 3. 轨迹求值 ---- */
  if (!ins_traj_run(&self->traj, arm_state, self->tick, &self->ref)) {
    self->fault_flags |= ARM_MPC_FAULT_TRAJ;
    enter_limp(self);
    return;
  }
  fill_unconnected_joints(self);
  ins_arm_set_feedback(&self->arm, self->q_actual);
  ec_arm_set_velocity_feedback(arm_state, self->dq_actual);
  ins_arm_update(&self->arm); /* 正解只给上层看末端位姿, 控制律不依赖它 */

  self->fault_flags |= check_tracking(self);
  if (self->fault_flags) {
    enter_limp(self);
    return;
  }
  self->arrived = target_arrived(self);

  /* ---- 4. RNEA 前馈 + MPC 错峰求解 ---- */
  cycle_start = DWT->CYCCNT;
  compute_rnea_feedforward(self);
  plan = ins_traj_plan(&self->traj);
  if (plan) mpc_staggered_solve(self, plan);
  self->cycles_last = DWT->CYCCNT - cycle_start;
  if (self->cycles_last > self->cycles_max) {
    self->cycles_max = self->cycles_last;
  }

  /* ---- 5. 输出 ---- */
  if (!output_torque(self)) {
    self->fault_flags |= ARM_MPC_FAULT_TORQUE;
    enter_limp(self);
  }
}
