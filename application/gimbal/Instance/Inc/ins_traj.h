#ifndef EC_DEMO_6DOF_INS_TRAJ_H
#define EC_DEMO_6DOF_INS_TRAJ_H

#include "ins_arm.h"

/*
 * 机械臂参考轨迹层。
 *
 * 职责: 把各路遥控器命令变成一份"可在任意时刻求值"的轨迹计划 (ec_traj.h),
 * 发布出去, 并在每个控制周期按当前时刻求值、把参考点写进 EC_Arm。
 *
 * 为什么要有这一层, 而不是让 cmd 层直接调 ec_traj:
 *   - 时间基准要统一。整个系统只认一个时钟 (osKernelSysTick, 1 kHz),
 *     tick 由调用者传进来, 本层不自己取时间, 因此可以在主机上测。
 *   - 命令合法性、限位、速度加速度上限要在一个地方检查。
 *   - 遥控器掉线、模式切换、故障需要一个统一的降级出口 (刹车而不是跳变)。
 *
 * 频率解耦: 命令什么频率来、本层什么频率被调用、MPC 什么频率求值,
 * 三者互不相干。命令来了就换一份计划; 消费者按自己的时刻去求值。
 * 中间没有采样队列, 所以不存在生产者/消费者速率匹配问题, 也没有
 * 欠载/过载的概念。
 *
 * 不在本层做的事:
 *   - 多种遥控器同时在线时的优先级仲裁。本层只记录 source_id 供上层判断,
 *     谁能发命令由上层决定。
 *   - 笛卡尔空间规划。逆解目前解不出来, 相关路径仍走 EC_Arm 原有实现。
 */

/*
 * 命令来源标识。本层不解释其含义, 只记录和透传, 供上层仲裁与诊断使用。
 * 这里给出的是建议取值, 上层可以自行扩展。
 */
#define INS_TRAJ_SOURCE_NONE 0u      /* 无来源 (上电初始状态、监督层自发动作) */
#define INS_TRAJ_SOURCE_HOST 1u      /* 上位机: 直接下发绝对目标角 */
#define INS_TRAJ_SOURCE_MANUAL 2u    /* 手动遥控: 下发位置增量 */
#define INS_TRAJ_SOURCE_SEQUENCE 3u  /* 自动流程: 多节点路径 */

/* 增量式遥控的默认固定平滑时长, 单位 s。 */
#define INS_TRAJ_DEFAULT_JOG_TIME 0.1f
/* 流式命令的默认超时, 单位 tick (1 tick = 1 ms)。 */
#define INS_TRAJ_DEFAULT_TIMEOUT_TICK 200u

typedef struct {
  EC_TrajChannel channel;       /* 双缓冲发布通道; 唯一的计划持有处。 */
  EC_TrajLimits limits;         /* 构造轨迹时使用的约束。 */
  float jog_smooth_time;        /* 增量式遥控的固定平滑时长, s。 */
  float jog_target[EC_MAX_JOINTS];
                                /* 增量累加出来的绝对目标, 单位 rad。 */
  uint32_t timeout_tick;        /* 流式命令的超时阈值, 单位 tick。 */
  uint32_t last_command_tick;   /* 最近一次成功接收命令的 tick。 */
  uint32_t reject_count;        /* 被拒绝的命令累计数, 诊断用。 */
  uint32_t timeout_count;       /* 因超时触发刹车的累计次数, 诊断用。 */
  uint8_t status;               /* 监督状态, 取值见 EC_TrajStatus。 */
  uint8_t active_source;        /* 当前计划的来源 source_id。 */
  uint8_t last_error;           /* 最近一次构造失败的 EC_TrajResult。 */
  bool streaming;               /* 当前计划是否来自持续发帧的来源。 */
} InsTraj;

/*
 * 初始化。q_init 为上电时的关节角 (通常取当前反馈), 初始计划是停在该点
 * 的 HOLD, 因此从第一个控制周期起就有合法的参考值可取, 不存在"还没收到
 * 命令所以没有参考值"的空窗。
 *
 * 限位取自 instance 的 MDH qmin/qmax; 速度与加速度上限取保守默认值,
 * 之后用 ins_traj_set_velocity_limit() 覆盖。
 */
bool ins_traj_init(InsTraj *traj, const InsArm *instance, const float *q_init,
                   uint32_t tick);

/* 覆盖速度与加速度上限, 单位 rad/s 与 rad/s^2; 数组长度为 6。 */
bool ins_traj_set_velocity_limit(InsTraj *traj, const float *dq_max,
                                 const float *ddq_max);
/* 设置增量式遥控的固定平滑时长, 单位 s。 */
bool ins_traj_set_jog_time(InsTraj *traj, float seconds);
/* 设置流式命令的超时 tick 数; 0 表示不检查超时。 */
void ins_traj_set_timeout(InsTraj *traj, uint32_t ticks);

/*
 * 每个控制周期调用一次。
 *
 * 顺序: 先跑监督 (超时检查、状态迁移), 再按 tick 求值, 最后把参考点写入
 * arm。out 可为 NULL, 不为 NULL 时额外拷出一份求值结果。
 *
 * 失败只发生在参数非法的情况; 正常运行时总能给出参考值, 因为计划走完后
 * 会自动保持在终点。
 */
bool ins_traj_run(InsTraj *traj, EC_Arm *arm, uint32_t tick,
                  EC_JointStateReference *out);

/* ------------------------------ 命令入口 ------------------------------ */

/*
 * 所有命令入口的共同行为:
 *   - 起点边界条件一律取"此刻的参考状态" (q, dq, ddq), 所以上一条轨迹没走完
 *     就接新命令不会在参考值上留台阶, 换遥控器同理。
 *   - 构造失败时旧计划保持不动, 只累加 reject_count 并记下 last_error;
 *     参考值永远不会因为一条坏命令而失效。
 *   - FAULT 状态下一律拒绝并返回 EC_TRAJ_ERR_ARG, 必须先 clear_fault。
 *   - tick 由调用方给, 用同一个时钟 (osKernelSysTick)。
 */

/*
 * 绝对目标角 (上位机风格): 从当前参考状态平滑到 q_target, 标称时长 duration。
 * 目标越限直接拒绝并返回 EC_TRAJ_ERR_LIMIT, 不做静默截断——上位机给了一个
 * 到不了的点, 悄悄改成别的点比报错更危险。
 */
EC_TrajResult ins_traj_command_absolute(InsTraj *traj, uint32_t tick,
                                        const float *q_target, float duration,
                                        uint8_t source_id);

/*
 * 位置增量 (当前 cmd 层 MouseKeySet 风格)。
 *
 * 每收到一帧就把增量累加到内部目标上, 然后从"此刻的参考状态"重新拟合一段
 * 固定时长 (jog_smooth_time) 的五次曲线。采样时刻总落在上一段内部, 所以
 * 起点带着非零速度和加速度, 拼接处 q/dq/ddq 全部连续。
 *
 * 这样处理的结果: 发帧频率 (14 ms 还是 50 ms) 不进入任何公式, 帧间抖动和
 * 偶尔丢帧都被自动吸收——丢一帧只是目标那一拍没往前挪, 曲线照常往当前目标
 * 收敛, 不会有速度台阶。代价是跟随比目标滞后大约一个平滑时长。
 *
 * 目标会被夹进关节限位 (增量式操作到了限位应该停在限位, 而不是报错)。
 */
EC_TrajResult ins_traj_command_increment(InsTraj *traj, uint32_t tick,
                                         const float *dq, uint8_t source_id);

/* 多节点路径: 依次经过 nodes[0..node_count-1], 第 i 段时长 durations[i]。 */
EC_TrajResult ins_traj_command_multi(InsTraj *traj, uint32_t tick,
                                     const float nodes[][EC_MAX_JOINTS],
                                     const float *durations,
                                     uint8_t node_count, uint8_t source_id);

/*
 * 兼容既有的 EC_ArmFsmCmd。只接受关节空间命令 (EC_PLAN_JOINT / EC_PLAN_NONE);
 * 笛卡尔命令返回 EC_TRAJ_ERR_ARG, 由调用方决定是否回退到 EC_Arm 原有路径。
 * EC_PLAN_NONE 会被当成"用最小允许时长走到该点", 而不是让参考值直接跳变。
 */
EC_TrajResult ins_traj_command_fsm(InsTraj *traj, uint32_t tick,
                                   const EC_ArmFsmCmd *command,
                                   uint8_t source_id);

/* 主动刹车: 按加速度上限减速到停。模式切换、遥控器切换都应该走这里。 */
EC_TrajResult ins_traj_brake(InsTraj *traj, uint32_t tick, uint8_t source_id);

/* 进入故障: 参考值保持在当前点, 状态置 FAULT, 不自动恢复。 */
void ins_traj_fault(InsTraj *traj, uint32_t tick);
/* 退出故障: 以当前反馈角为新的保持点, 状态回到 HOLD。 */
void ins_traj_clear_fault(InsTraj *traj, uint32_t tick, const float *q_now);

/* ------------------------------ 读取 ------------------------------ */

/*
 * 取当前发布的计划。这就是本层对 MPC 的全部接口: 拿到计划以后, 用
 * ec_traj_eval_joint_time() 在预测时域内的任意时刻求值即可, 既不需要
 * 采样队列, 也不要求两边频率一致。
 */
const EC_TrajPlan *ins_traj_plan(const InsTraj *traj);

/* 当前监督状态。 */
uint8_t ins_traj_status(const InsTraj *traj);

#endif /* EC_DEMO_6DOF_INS_TRAJ_H */
