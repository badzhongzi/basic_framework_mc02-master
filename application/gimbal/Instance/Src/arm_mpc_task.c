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

/*
 * MPC 的一拍 = 一个控制周期 = INS_ARM_CONTROL_TICK_MS 个 tick (轨迹计划的 tick
 * 就是 osKernelSysTick, 1 ms)。生成代码里的 MPC_BASE_DT 必须等于控制周期,
 * 在 ArmMpcTaskInit() 里检查 (浮点比较不能写成静态断言)。
 */
#define ARM_MPC_TICKS_PER_STEP INS_ARM_CONTROL_TICK_MS

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

/* ---------- 前馈 ---------- */

/*
 * 前馈 = RNEA(实际 q, 实际 dq, 力矩生效时刻的参考 ddq) + 摩擦补偿(实际 dq)。
 * 前馈要抵消的是真实动力学, 而 ddq 是"希望关节产生的加速度"; 传参考 q/dq
 * 进去就变成纯开环前馈。摩擦补偿必须用生成代码里的 mpc_friction_ff(), 它和
 * 求解器里总力矩约束用的 tau_ff 是同一个东西。
 */
static void compute_feedforward(ArmMpcTask *self, const float *ddq_ref) {
  const casadi_real *arg[rnea_SZ_ARG] = {self->q_actual, self->dq_actual,
                                         ddq_ref};
  casadi_real *res[rnea_SZ_RES] = {self->tau_ff};
  float fric[ARM_MPC_JOINT_COUNT];
  uint8_t j;

  /* rnea_SZ_IW = rnea_SZ_W = 0, 不需要工作内存。 */
  (void)rnea(arg, res, NULL, NULL, 0);
  mpc_friction_ff(self->dq_actual, fric);
  for (j = 0; j < ARM_MPC_JOINT_COUNT; ++j) self->tau_ff[j] += fric[j];
}

/* 参考前馈 tref = RNEA(参考 q, dq, ddq) + 摩擦补偿(参考 dq), 只用来预测
 * 时域末尾的前馈 (总力矩约束沿时域变化)。 */
static void reference_torque(const EC_JointStateReference *r,
                             float out[ARM_MPC_JOINT_COUNT]) {
  const casadi_real *arg[rnea_SZ_ARG] = {r->q, r->q_dot, r->q_ddot};
  casadi_real *res[rnea_SZ_RES] = {out};
  float fric[ARM_MPC_JOINT_COUNT];
  uint8_t j;

  (void)rnea(arg, res, NULL, NULL, 0);
  mpc_friction_ff(r->q_dot, fric);
  for (j = 0; j < ARM_MPC_JOINT_COUNT; ++j) out[j] += fric[j];
}

/* ---------- MPC: 观测器 + 每拍全部关节求解 ---------- */

/*
 * 和 Python 仿真 sim_core.run() 的循环体、PC 回归 ctest/test_loop.c 逐行对应,
 * 调用顺序和时刻不要改 (说明文档 4.13)。
 *
 *   t_meas  反馈对应的时刻 = 现在 - MPC_EST_FB 拍, 观测器用它的参考算误差
 *   t_apply 力矩真正生效的时刻 = 现在 + MPC_EST_CT 拍, 前馈和 MPC 从这里开始预测
 *
 * 1. 前馈: RNEA(实际 q, dq, ddq_ref(t_apply)) + 摩擦补偿
 * 2. 时域末尾的前馈预测 tau_ff_end = tau_ff + tref(t_apply + NS 拍) - tref(t_apply)
 * 3. 每个约束检查点的参考 (位置/速度限位要换成误差边界)
 * 4. 惯量比 rho (P3), 观测器在测量时刻更新 (P1, 所有关节)
 * 5. 每个关节: 观测器前推到 t_apply 得到锚点和扰动估计 (P2), 定迭代 ADMM 求 u
 * 6. dtau = rho * u, 按 effort 饱和
 * 观测器的 commit 要用真正发出去的量, 在输出之后做 (commit_applied_torque)。
 *
 * 未接电机的关节按理想跟踪, 测量误差恒为 0、不求解、修正量为 0, 观测器就一直
 * 停在 0, 不会把 "算出来但没发出去" 的修正量当成扰动积分进去。
 *
 * 返回 false 表示轨迹求值失败。
 */
_Static_assert(MPC_NJ == ARM_MPC_JOINT_COUNT, "MPC joint count mismatch");

static bool mpc_solve_all(ArmMpcTask *self, const EC_TrajPlan *plan) {
  const uint32_t step = ARM_MPC_TICKS_PER_STEP;
  const uint32_t t_meas = self->tick - (uint32_t)MPC_EST_FB * step;
  const uint32_t t_apply = self->tick + (uint32_t)MPC_EST_CT * step;
  EC_JointStateReference r_meas, r_apply, r_end, r_e;
  float tref0[ARM_MPC_JOINT_COUNT], tref1[ARM_MPC_JOINT_COUNT];
  float tau_ff_end[ARM_MPC_JOINT_COUNT];
  float dtau[ARM_MPC_JOINT_COUNT], tau_cmd[ARM_MPC_JOINT_COUNT];
  float qr_e[MPC_CS_NB][ARM_MPC_JOINT_COUNT];
  float dqr_e[MPC_CS_NB][ARM_MPC_JOINT_COUNT];
  uint8_t i, j;

  if (!ec_traj_eval_tick(plan, t_meas, &r_meas) ||
      !ec_traj_eval_tick(plan, t_apply, &r_apply) ||
      !ec_traj_eval_tick(plan, t_apply + (uint32_t)MPC_CS_NS * step, &r_end)) {
    return false;
  }
  for (i = 0; i < MPC_CS_NB; ++i) {
    if (!ec_traj_eval_tick(plan, t_apply + (uint32_t)MPC_CS_ENDS[i] * step,
                           &r_e)) {
      return false;
    }
    for (j = 0; j < ARM_MPC_JOINT_COUNT; ++j) {
      qr_e[i][j] = r_e.q[j];
      dqr_e[i][j] = r_e.q_dot[j];
    }
  }

  /* 1-2. 前馈和时域末尾的前馈预测 */
  compute_feedforward(self, r_apply.q_ddot);
  reference_torque(&r_apply, tref0);
  reference_torque(&r_end, tref1);
  for (j = 0; j < ARM_MPC_JOINT_COUNT; ++j) {
    tau_ff_end[j] = self->tau_ff[j] + (tref1[j] - tref0[j]);
  }

  /* 4. 惯量比; 观测器在测量时刻更新 */
  mpc_inertia_ratio(self->q_actual, self->rho);
  for (j = 0; j < ARM_MPC_JOINT_COUNT; ++j) {
    if (self->motors[j]) {
      mpc_est_update((int)j, r_meas.q[j] - self->q_actual[j],
                     r_meas.q_dot[j] - self->dq_actual[j]);
    } else {
      mpc_est_update((int)j, 0.0f, 0.0f);
    }
  }

  /* 5. 每个关节: 前推锚点 + 求解 */
  for (j = 0; j < ARM_MPC_JOINT_COUNT; ++j) {
    float anchor[2];
    float w_hat = 0.0f;
    float qe[MPC_CS_NB], dqe[MPC_CS_NB];
    float u = 0.0f;

    mpc_est_anchor((int)j, anchor, &w_hat);
    self->w_hat[j] = w_hat;
    if (self->motors[j]) {
      for (i = 0; i < MPC_CS_NB; ++i) {
        qe[i] = qr_e[i][j];
        dqe[i] = dqr_e[i][j];
      }
      u = mpc_cs_step((int)j, anchor, w_hat, self->tau_ff[j], tau_ff_end[j],
                      self->rho[j], qe, dqe);
    }
    dtau[j] = self->rho[j] * u;
  }

  /* 6. 按 effort 饱和; 修正量记成饱和之后实际加上去的部分 */
  (void)mpc_saturate_total(self->tau_ff, dtau, tau_cmd);
  for (j = 0; j < ARM_MPC_JOINT_COUNT; ++j) {
    self->mpc_dtau[j] = tau_cmd[j] - self->tau_ff[j];
  }
  return true;
}

/*
 * 告诉观测器这一拍真正发出去的修正量 u = (tau_total - tau_ff) / rho。
 * 必须用 output_torque() 按 tau_max 截完以后的值: 观测器只认实际作用的量,
 * 积分作用才不会在饱和时越积越多。未接的关节发出去的修正量是 0。
 */
static void commit_applied_torque(ArmMpcTask *self) {
  uint8_t j;
  for (j = 0; j < ARM_MPC_JOINT_COUNT; ++j) {
    float u = 0.0f;
    if (self->motors[j] && self->rho[j] > 0.0f) {
      u = (self->tau_total[j] - self->tau_ff[j]) / self->rho[j];
    }
    mpc_est_commit((int)j, u);
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
 * 总力矩 = 前馈 + MPC 修正, 按关节力矩上限饱和后直接以 N·m 下发。
 * 求解器已经保证 |tau_ff + dtau| 不超过生成代码里的 effort (MPC_EFFORT_MAX),
 * 这里按 joint_cfg 的 tau_max 再截一次, 是调试阶段更保守的上限, 也是最后一道
 * 保险。tau_max 在初始化时已经和电机的 T_MAX 取过小值。
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
  /* 零力矩期间观测器和求解器都没跑 (实际发出去的修正量是 -tau_ff, 喂进去会被
   * 当成巨大的扰动), 恢复出力时从零开始, 和仿真的初始状态一致。 */
  mpc_est_reset();
  mpc_cs_reset();
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
  /* 生成代码的离散步长必须等于控制周期, 否则预测、观测器、延迟补偿全错 */
  if (fabsf(MPC_BASE_DT - INS_ARM_CONTROL_PERIOD) > 1e-6f) return false;
  mpc_est_reset();
  mpc_cs_reset();

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

  /* ---- 4. 前馈 + 观测器 + MPC (每拍全部关节) ---- */
  cycle_start = DWT->CYCCNT;
  plan = ins_traj_plan(&self->traj);
  if (!plan || !mpc_solve_all(self, plan)) {
    self->fault_flags |= ARM_MPC_FAULT_TRAJ;
    enter_limp(self);
    return;
  }
  self->cycles_last = DWT->CYCCNT - cycle_start;
  if (self->cycles_last > self->cycles_max) {
    self->cycles_max = self->cycles_last;
  }

  /* ---- 5. 输出, 再把真正发出去的量告诉观测器 ---- */
  if (!output_torque(self)) {
    self->fault_flags |= ARM_MPC_FAULT_TORQUE;
    enter_limp(self);
    return;
  }
  commit_applied_torque(self);
}
