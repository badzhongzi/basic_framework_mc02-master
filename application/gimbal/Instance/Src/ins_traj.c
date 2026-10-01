#include "ins_traj.h"

#include <math.h>
#include <string.h>

/*
 * 一个 tick 的秒数。时间基准是 osKernelSysTick(), FreeRTOSConfig.h 里
 * configTICK_RATE_HZ = 1000, 所以 1 tick = 1 ms。
 *
 * 这里刻意不使用 INS_ARM_CONTROL_PERIOD: 控制周期是"本层被调用的间隔",
 * tick 周期是"时间刻度", 两者是彼此独立的量。把它们混成一个数, 就又回到了
 * "轨迹进度取决于被调用过几次"的老问题——调用频率一变, 整条轨迹的时间标尺
 * 跟着变。现在无论本层 1 ms 还是 10 ms 被调用一次, 同一时刻求值结果不变。
 */
#define INS_TRAJ_TICK_PERIOD 0.001f

/*
 * 段时长的上下界。下界是十个控制周期, 免得生成一段短到只被采样几次的曲线
 * (采样点太少时电机看到的实际上是阶跃); 上界给"时长被自动拉长"一个尽头,
 * 拉到这里还不可行就报错, 而不是无限拉下去。
 */
#define INS_TRAJ_MIN_SEG_TIME (10.0f * INS_ARM_CONTROL_PERIOD)
#define INS_TRAJ_MAX_SEG_TIME 5.0f

/*
 * 默认速度/加速度上限。dq 取 openarm.urdf 里 revolute 关节的 velocity 限值;
 * ddq 的 URDF 没给, 这个数是估的——真实值要么实测, 要么由力矩上限和惯量反推。
 * 上限偏小只会让动作变慢, 偏大则会让参考轨迹跑到执行机构跟不上的地方,
 * 所以先给保守值, 标定后用 ins_traj_set_velocity_limit() 覆盖。
 */
#define INS_TRAJ_DEFAULT_DQ_MAX 1.0f
#define INS_TRAJ_DEFAULT_DDQ_MAX 10.0f

static bool traj_finite_array(const float *v, uint8_t count) {
  uint8_t j;
  if (!v) return false;
  for (j = 0; j < count; ++j) {
    if (!isfinite(v[j])) return false;
  }
  return true;
}

/*
 * 取"此刻的参考状态"。所有新命令都以它作为起点边界条件, 这是拼接处
 * q/dq/ddq 连续的全部来源: 上一条轨迹走到哪, 新轨迹就从那个位置、那个速度、
 * 那个加速度接上, 所以中途换命令、换遥控器都不会在参考值上留台阶。
 */
static bool traj_current_state(const InsTraj *traj, uint32_t tick,
                               EC_JointStateReference *out) {
  const EC_TrajPlan *plan = ec_traj_channel_read(&traj->channel);
  return plan != NULL && ec_traj_eval_tick(plan, tick, out);
}

static EC_TrajResult traj_reject(InsTraj *traj, EC_TrajResult rc) {
  traj->last_error = (uint8_t)rc;
  ++traj->reject_count;
  return rc;
}

/* 提交备用槽并同步本层的簿记字段。计划一旦 commit, 消费者下一次 read()
 * 就会看到新的一份, 所以簿记必须紧跟其后, 不能隔着别的逻辑。 */
static void traj_accept(InsTraj *traj, uint8_t status, uint8_t source_id,
                        bool streaming) {
  ec_traj_channel_commit(&traj->channel);
  traj->status = status;
  traj->active_source = source_id;
  traj->streaming = streaming;
  traj->last_error = (uint8_t)EC_TRAJ_OK;
}

/* 命令入口的公共前置检查。故障状态下一律拒绝: 故障的语义是"现在不该再动",
 * 谁来下命令都一样, 要恢复必须显式调用 ins_traj_clear_fault()。 */
static bool traj_intake_ready(const InsTraj *traj) {
  return traj != NULL && traj->status != (uint8_t)EC_TRAJ_FAULT;
}

static EC_TrajResult traj_publish_p2p(InsTraj *traj, uint32_t tick,
                                      const EC_JointStateReference *start,
                                      const float *q_target, float duration,
                                      uint8_t source_id, bool streaming) {
  EC_TrajPlan *slot = ec_traj_channel_begin_write(&traj->channel);
  const EC_TrajResult rc = ec_traj_plan_point_to_point(
      slot, &traj->limits, start, q_target, duration, tick,
      INS_TRAJ_TICK_PERIOD, source_id, NULL);

  if (rc != EC_TRAJ_OK) return traj_reject(traj, rc);
  traj->last_command_tick = tick;
  traj_accept(traj, (uint8_t)EC_TRAJ_ACTIVE, source_id, streaming);
  return EC_TRAJ_OK;
}

static EC_TrajResult traj_publish_brake(InsTraj *traj, uint32_t tick,
                                        const EC_JointStateReference *start,
                                        uint8_t source_id) {
  EC_TrajPlan *slot = ec_traj_channel_begin_write(&traj->channel);
  const EC_TrajResult rc = ec_traj_plan_brake(
      slot, &traj->limits, start, tick, INS_TRAJ_TICK_PERIOD, source_id);

  if (rc != EC_TRAJ_OK) return traj_reject(traj, rc);
  traj_accept(traj, (uint8_t)EC_TRAJ_BRAKING, source_id, false);
  return EC_TRAJ_OK;
}

static void traj_publish_hold(InsTraj *traj, uint32_t tick, const float *q_hold,
                              uint8_t status, uint8_t source_id) {
  float q[EC_MAX_JOINTS];
  EC_TrajPlan *slot;
  uint8_t j;

  /* 先拷一份: q_hold 通常指向当前可读槽, 而 begin_write() 给的是另一个槽,
   * 两者不重叠; 但调用方也可能直接传进来一个即将被覆盖的缓冲, 拷贝一次
   * 把这类别名问题一次性排除掉。 */
  for (j = 0; j < traj->limits.joint_count; ++j) q[j] = q_hold[j];
  slot = ec_traj_channel_begin_write(&traj->channel);
  if (!ec_traj_plan_hold(slot, traj->limits.joint_count, q, tick,
                         INS_TRAJ_TICK_PERIOD, status, source_id)) {
    return;
  }
  traj_accept(traj, status, source_id, false);
}

/* 计划是否已经走完。用整数 tick 差再折算成秒, 不累加浮点时间。 */
static bool traj_plan_done(const EC_TrajPlan *plan, uint32_t tick) {
  const int32_t delta = (int32_t)(tick - plan->t0_tick);
  return (float)delta * plan->tick_period >= plan->total_duration;
}

/*
 * 监督。每个控制周期跑一次, 负责两件事: 流式来源掉线时刹车, 轨迹走完时
 * 把状态收回 HOLD。
 */
static void traj_supervise(InsTraj *traj, uint32_t tick) {
  const EC_TrajPlan *plan;

  if (traj->status == (uint8_t)EC_TRAJ_FAULT ||
      traj->status == (uint8_t)EC_TRAJ_HOLD) {
    return;
  }

  /*
   * 掉线判断只对流式来源做。绝对目标和多节点命令是一次性的: 走完自然停在
   * 终点, "不再发新命令"是正常状态而不是掉线; 反过来, 增量式遥控本来就靠
   * 连续发帧维持运动, 帧停了就必须主动减速, 否则参考值会按最后那段曲线继续
   * 跑到一个遥控器已经不再确认的位置。
   *
   * 注意这条通路在默认参数下几乎不会触发: 平滑时长 100 ms 远小于超时
   * 200 ms, 帧一停, 下面"轨迹走完"那一支会先把状态收回 HOLD, 而那一段的
   * 终点速度本来就是 0, 已经停住了。真正需要这条通路的是平滑时长或段时长
   * 大于超时阈值的配置, 它是兜底而不是主路径。
   */
  if (traj->status == (uint8_t)EC_TRAJ_ACTIVE && traj->streaming &&
      traj->timeout_tick > 0u &&
      (uint32_t)(tick - traj->last_command_tick) >= traj->timeout_tick) {
    EC_JointStateReference start;
    ++traj->timeout_count;
    if (traj_current_state(traj, tick, &start)) {
      (void)traj_publish_brake(traj, tick, &start, traj->active_source);
    }
    return;
  }

  plan = ec_traj_channel_read(&traj->channel);
  if (plan != NULL && traj_plan_done(plan, tick)) {
    /* 轨迹走完。求值本身已经自动保持在终点, 换一份纯保持计划只是为了让
     * 计划里的 status 字段对消费者保持诚实——MPC 要靠它判断当前该不该跟随,
     * 一份 status 永远是 ACTIVE 却其实早就停了的计划会误导它。 */
    traj_publish_hold(traj, tick, plan->q_hold, (uint8_t)EC_TRAJ_HOLD,
                      traj->active_source);
  }
}

bool ins_traj_init(InsTraj *traj, const InsArm *instance, const float *q_init,
                   uint32_t tick) {
  const EC_Arm *arm;
  uint8_t j;

  if (!traj || !instance) return false;
  arm = ins_arm_const_state(instance);
  if (!arm || !arm->initialized || arm->joint_count == 0u) return false;
  if (!traj_finite_array(q_init, arm->joint_count)) return false;

  memset(traj, 0, sizeof(*traj));
  if (!ec_traj_limits_init(&traj->limits, arm->joint_count)) return false;
  for (j = 0; j < arm->joint_count; ++j) {
    /* 限位直接取 EC_Arm 的软限位 (也就是 MDH 表里的 qmin/qmax), 不在这里
     * 另写一份。同一个物理限位写在两处, 早晚会漂开——ins_arm.c 的控制周期
     * 和 ins_planner.h 的那份就正好是这么漂的。 */
    traj->limits.q_min[j] = arm->joint_min[j];
    traj->limits.q_max[j] = arm->joint_max[j];
    traj->limits.dq_max[j] = INS_TRAJ_DEFAULT_DQ_MAX;
    traj->limits.ddq_max[j] = INS_TRAJ_DEFAULT_DDQ_MAX;
    traj->jog_target[j] = q_init[j];
  }
  traj->limits.min_duration = INS_TRAJ_MIN_SEG_TIME;
  traj->limits.max_duration = INS_TRAJ_MAX_SEG_TIME;
  traj->jog_smooth_time = INS_TRAJ_DEFAULT_JOG_TIME;
  traj->timeout_tick = INS_TRAJ_DEFAULT_TIMEOUT_TICK;
  traj->status = (uint8_t)EC_TRAJ_HOLD;
  traj->active_source = INS_TRAJ_SOURCE_NONE;
  traj->last_command_tick = tick;

  /* 上电即发布一份停在 q_init 的保持计划: 从第一个控制周期起就有合法的
   * 参考值可取, 不存在"还没收到命令所以参考值是 0"的空窗——那个空窗会让
   * 机械臂在上电瞬间往零位冲。 */
  return ec_traj_channel_init(&traj->channel, arm->joint_count, q_init, tick,
                              INS_TRAJ_TICK_PERIOD);
}

bool ins_traj_set_velocity_limit(InsTraj *traj, const float *dq_max,
                                 const float *ddq_max) {
  uint8_t j;

  if (!traj) return false;
  if (!traj_finite_array(dq_max, traj->limits.joint_count)) return false;
  if (!traj_finite_array(ddq_max, traj->limits.joint_count)) return false;
  for (j = 0; j < traj->limits.joint_count; ++j) {
    if (dq_max[j] <= 0.0f || ddq_max[j] <= 0.0f) return false;
  }
  for (j = 0; j < traj->limits.joint_count; ++j) {
    traj->limits.dq_max[j] = dq_max[j];
    traj->limits.ddq_max[j] = ddq_max[j];
  }
  return true;
}

bool ins_traj_set_jog_time(InsTraj *traj, float seconds) {
  if (!traj || !isfinite(seconds)) return false;
  if (seconds < traj->limits.min_duration ||
      seconds > traj->limits.max_duration) {
    return false;
  }
  traj->jog_smooth_time = seconds;
  return true;
}

void ins_traj_set_timeout(InsTraj *traj, uint32_t ticks) {
  if (traj) traj->timeout_tick = ticks;
}

bool ins_traj_run(InsTraj *traj, EC_Arm *arm, uint32_t tick,
                  EC_JointStateReference *out) {
  const EC_TrajPlan *plan;
  EC_JointStateReference state;

  if (!traj) return false;

  traj_supervise(traj, tick);

  plan = ec_traj_channel_read(&traj->channel);
  if (!plan || !ec_traj_eval_tick(plan, tick, &state)) return false;
  if (out) *out = state;
  /* 写进 EC_Arm 用 set_state_reference 而不是 generate_reference: 后者是
   * 逐周期在线插值, 会把自己的时间往前推一格; 这里的时间完全由 tick 决定,
   * 两条路径在同一个周期里只能走一条。 */
  if (arm && !ec_arm_set_state_reference(arm, &state)) return false;
  return true;
}

EC_TrajResult ins_traj_command_absolute(InsTraj *traj, uint32_t tick,
                                        const float *q_target, float duration,
                                        uint8_t source_id) {
  EC_JointStateReference start;

  if (!traj_intake_ready(traj)) return EC_TRAJ_ERR_ARG;
  if (!traj_finite_array(q_target, traj->limits.joint_count) ||
      !isfinite(duration) || duration <= 0.0f) {
    return traj_reject(traj, EC_TRAJ_ERR_ARG);
  }
  if (!traj_current_state(traj, tick, &start)) return EC_TRAJ_ERR_ARG;
  /* 越限的目标由 ec_traj_plan_point_to_point() 直接拒绝, 本层不做静默截断:
   * 上位机给了一个到不了的点, 悄悄改成另一个点比报错更危险, 因为上位机会
   * 以为自己的目标被接受了。 */
  return traj_publish_p2p(traj, tick, &start, q_target, duration, source_id,
                          false);
}

EC_TrajResult ins_traj_command_increment(InsTraj *traj, uint32_t tick,
                                         const float *dq, uint8_t source_id) {
  EC_JointStateReference start;
  uint8_t n;
  uint8_t j;
  bool reseed;

  if (!traj_intake_ready(traj)) return EC_TRAJ_ERR_ARG;
  n = traj->limits.joint_count;
  if (!traj_finite_array(dq, n)) return traj_reject(traj, EC_TRAJ_ERR_ARG);
  if (!traj_current_state(traj, tick, &start)) return EC_TRAJ_ERR_ARG;

  /*
   * 换了来源, 或者上一段已经结束/正在刹车/处于保持, 都说明累加器里的目标
   * 已经和实际参考值脱节, 必须重新以当前参考点为基准。否则第一帧会被解释成
   * "从那个旧目标继续往前", 参考值瞬间跳过中间那一大截。
   */
  reseed = (traj->active_source != source_id) ||
           (traj->status != (uint8_t)EC_TRAJ_ACTIVE);
  if (reseed) {
    for (j = 0; j < n; ++j) traj->jog_target[j] = start.q[j];
  }
  for (j = 0; j < n; ++j) {
    /* 增量式操作撞到限位应该停在限位, 而不是整帧拒绝: 手动遥控时"报错"没有
     * 任何可见反馈, 只会表现成推了一下整条臂都不动。 */
    traj->jog_target[j] = ec_limit(traj->jog_target[j] + dq[j],
                                   traj->limits.q_min[j],
                                   traj->limits.q_max[j]);
  }
  return traj_publish_p2p(traj, tick, &start, traj->jog_target,
                          traj->jog_smooth_time, source_id, true);
}

EC_TrajResult ins_traj_command_multi(InsTraj *traj, uint32_t tick,
                                     const float nodes[][EC_MAX_JOINTS],
                                     const float *durations,
                                     uint8_t node_count, uint8_t source_id) {
  EC_JointStateReference start;
  EC_TrajPlan *slot;
  EC_TrajResult rc;
  uint8_t i;

  if (!traj_intake_ready(traj)) return EC_TRAJ_ERR_ARG;
  if (!nodes || !durations || node_count == 0u) {
    return traj_reject(traj, EC_TRAJ_ERR_ARG);
  }
  if (node_count > (uint8_t)EC_TRAJ_MAX_SEG) {
    return traj_reject(traj, EC_TRAJ_ERR_SEG);
  }
  for (i = 0; i < node_count; ++i) {
    if (!traj_finite_array(nodes[i], traj->limits.joint_count) ||
        !isfinite(durations[i]) || durations[i] <= 0.0f) {
      return traj_reject(traj, EC_TRAJ_ERR_ARG);
    }
  }
  if (!traj_current_state(traj, tick, &start)) return EC_TRAJ_ERR_ARG;

  slot = ec_traj_channel_begin_write(&traj->channel);
  rc = ec_traj_plan_multi_point(slot, &traj->limits, &start, nodes, durations,
                                node_count, tick, INS_TRAJ_TICK_PERIOD,
                                source_id);
  if (rc != EC_TRAJ_OK) return traj_reject(traj, rc);
  traj->last_command_tick = tick;
  traj_accept(traj, (uint8_t)EC_TRAJ_ACTIVE, source_id, false);
  return EC_TRAJ_OK;
}

EC_TrajResult ins_traj_command_fsm(InsTraj *traj, uint32_t tick,
                                   const EC_ArmFsmCmd *command,
                                   uint8_t source_id) {
  float nodes[EC_TRAJ_MAX_SEG][EC_MAX_JOINTS];
  float durations[EC_TRAJ_MAX_SEG];
  uint8_t n;
  uint8_t i;
  uint8_t j;

  if (!traj_intake_ready(traj) || !command) return EC_TRAJ_ERR_ARG;
  if (command->plan_type != EC_PLAN_JOINT &&
      command->plan_type != EC_PLAN_NONE) {
    /* 笛卡尔命令本层不接。逆解目前解不出来, 而且笛卡尔轨迹要每周期在线求
     * 一次逆解, 那是 EC_Arm 原有路径的形态, 不是"一份可求值系数"的形态。
     * 调用方拿到这个返回码可以自行回退到 ec_arm_set_command()。 */
    return EC_TRAJ_ERR_ARG;
  }
  n = command->node_count;
  if (n == 0u) return traj_reject(traj, EC_TRAJ_ERR_ARG);
  if (n > (uint8_t)EC_TRAJ_MAX_SEG) return traj_reject(traj, EC_TRAJ_ERR_SEG);
  /* EC_PLAN_NONE 在 EC_Arm 里只用 nodes[0]。 */
  if (command->plan_type == EC_PLAN_NONE) n = 1u;

  for (i = 0; i < n; ++i) {
    if (command->nodes[i].workspace != EC_WORKSPACE_JOINT) {
      return traj_reject(traj, EC_TRAJ_ERR_ARG);
    }
    for (j = 0; j < traj->limits.joint_count; ++j) {
      nodes[i][j] = command->nodes[i].joint[j];
    }
    /* EC_PLAN_NONE 的原语义是"参考值直接跳到该点"。这里改成用最小允许时长
     * 走过去: 目标语义不变, 但不会给电机一个位置台阶, 也不会让 q_dot 出现
     * 一个除以控制周期得来的尖峰。 */
    durations[i] = (command->plan_type == EC_PLAN_NONE)
                       ? traj->limits.min_duration
                       : command->traj_time[i];
    if (!isfinite(durations[i]) || durations[i] <= 0.0f) {
      durations[i] = traj->limits.min_duration;
    }
  }
  return ins_traj_command_multi(
      traj, tick, (const float (*)[EC_MAX_JOINTS])nodes, durations, n,
      source_id);
}

EC_TrajResult ins_traj_brake(InsTraj *traj, uint32_t tick, uint8_t source_id) {
  EC_JointStateReference start;

  if (!traj_intake_ready(traj)) return EC_TRAJ_ERR_ARG;
  if (!traj_current_state(traj, tick, &start)) return EC_TRAJ_ERR_ARG;
  return traj_publish_brake(traj, tick, &start, source_id);
}

void ins_traj_fault(InsTraj *traj, uint32_t tick) {
  EC_JointStateReference start;

  if (!traj || traj->status == (uint8_t)EC_TRAJ_FAULT) return;
  if (!traj_current_state(traj, tick, &start)) return;
  /* 故障时参考值停在当前点, 而不是走一条刹车曲线: 刹车的前提是执行机构还
   * 受控, 而故障的前提恰恰是这一点不成立。参考值仍然可求值, 力矩下不下发
   * 由上层看 status 决定。 */
  traj_publish_hold(traj, tick, start.q, (uint8_t)EC_TRAJ_FAULT,
                    traj->active_source);
}

void ins_traj_clear_fault(InsTraj *traj, uint32_t tick, const float *q_now) {
  uint8_t j;

  if (!traj || traj->status != (uint8_t)EC_TRAJ_FAULT) return;
  if (!traj_finite_array(q_now, traj->limits.joint_count)) return;
  /* 以当前反馈角为新的保持点。用反馈而不是故障前的参考值: 故障期间机械臂
   * 可能已经被重力拉走或被人推动, 拿旧参考值恢复等于让它先跳回去。 */
  traj->status = (uint8_t)EC_TRAJ_HOLD;
  traj_publish_hold(traj, tick, q_now, (uint8_t)EC_TRAJ_HOLD,
                    INS_TRAJ_SOURCE_NONE);
  for (j = 0; j < traj->limits.joint_count; ++j) traj->jog_target[j] = q_now[j];
  traj->last_command_tick = tick;
}

const EC_TrajPlan *ins_traj_plan(const InsTraj *traj) {
  return traj ? ec_traj_channel_read(&traj->channel) : NULL;
}

uint8_t ins_traj_status(const InsTraj *traj) {
  return traj ? traj->status : (uint8_t)EC_TRAJ_FAULT;
}
