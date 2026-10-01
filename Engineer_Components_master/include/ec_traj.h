#ifndef ENGINEER_COMPONENTS_C_TRAJ_H
#define ENGINEER_COMPONENTS_C_TRAJ_H

#include "ec_types.h"

/*
 * 可求值轨迹计划 (evaluable trajectory plan)。
 *
 * 与 ec_planner.h 的区别, 也是本模块存在的理由:
 *
 *   EC_JointPlanner 是"逐周期在线插值"——每调用一次 ec_joint_planner_plan_state()
 *   内部 elapsed 前进一个控制周期, 吐出一个参考点。它只能回答"下一拍是多少",
 *   而且问一次就把自己的时间往前推一次。
 *
 *   EC_TrajPlan 是一个纯数据对象: 一组五次多项式系数 + 一个绝对时间起点。
 *   它能回答"任意时刻 t 是多少", 问多少次、按什么顺序问、问未来还是问过去,
 *   结果都一样, 也不会改变自身状态。
 *
 * 这个区别是 MPC 能接进来的前提。MPC 每个求解周期需要预测时域内 N 个点
 * (t0, t0+dt, ..., t0+N*dt) 的 q/dq/ddq; 逐周期插值器给不了, 而这里
 * 一次求值一个点、求 N 次即可, 且不需要在生产者和消费者之间放采样队列,
 * 也就不存在两端频率必须匹配的问题。上层命令频率、规划频率、MPC 频率、
 * 控制频率四者彼此无关: 命令来了就换一份计划, 谁想要参考值谁自己按自己的
 * 时间基准去求值。
 *
 * 设计约束:
 *   - 纯计算, 不依赖操作系统、不含全局变量、所有函数可重入。
 *     时间基准由调用者以整数 tick 传入 (工程里用 osKernelSysTick(), 1 kHz)。
 *   - 不做堆分配, 结构体可静态分配。
 *   - 求值路径上不做限幅、不做角度归一化、不做任何有副作用的修正。
 *     所有安全检查都在构造计划时一次性做完 (见 ec_traj_plan_* 系列),
 *     因为 MPC 的 QP 里没有状态约束, 参考层是唯一的防线; 而在求值路径上
 *     限幅会让"同一时刻求两次得到不同结果", 直接破坏 MPC 的时域一致性。
 *
 * 角度单位 rad, 时间单位 s, 与库内其余部分一致。
 */

/* 单份计划内的最大分段数。多节点命令超过此数会被拒绝。 */
#ifndef EC_TRAJ_MAX_SEG
#define EC_TRAJ_MAX_SEG 8u
#endif

/* 可行性检查时在每段上均匀采样的点数 (含两端), 只在构造计划时使用。 */
#ifndef EC_TRAJ_CHECK_SAMPLES
#define EC_TRAJ_CHECK_SAMPLES 17u
#endif

/* 不可行时拉长段时长的最大重试次数。 */
#ifndef EC_TRAJ_STRETCH_ITERS
#define EC_TRAJ_STRETCH_ITERS 6u
#endif

/* 轨迹层的监督状态。计划本身携带状态, 便于消费者一眼看出该不该跟随。 */
typedef enum {
  EC_TRAJ_HOLD = 0, /* 停在 q_hold 不动, dq/ddq 恒为 0。上电后的初始状态。 */
  EC_TRAJ_ACTIVE,   /* 正常跟随分段轨迹。 */
  EC_TRAJ_BRAKING,  /* 正在按加速度上限减速到停; 走完后等价于 HOLD。 */
  EC_TRAJ_FAULT     /* 故障。参考值仍然可求值 (停在 q_hold), 供上层决定是否下发力矩。 */
} EC_TrajStatus;

/* 构造类接口的返回码。求值类接口返回 bool。 */
typedef enum {
  EC_TRAJ_OK = 0,
  EC_TRAJ_ERR_ARG,        /* 空指针、关节数越界、时长非正等参数错误。 */
  EC_TRAJ_ERR_SEG,        /* 节点数超过 EC_TRAJ_MAX_SEG。 */
  EC_TRAJ_ERR_LIMIT,      /* 目标点本身就在关节限位之外。 */
  EC_TRAJ_ERR_INFEASIBLE  /* 拉伸到 max_duration 仍然突破速度/加速度/限位。 */
} EC_TrajResult;

/*
 * 一段五次多项式。
 *
 * 自变量是归一化时间 s = t_local / duration, s 属于 [0, 1]:
 *   q(s)   = sum_{k=0..5} c[j][k] * s^k
 *   dq(t)  = q'(s)  / duration
 *   ddq(t) = q''(s) / duration^2
 *
 * 用归一化时间而不是直接用秒, 是为了让系数量级停在 O(delta_q) 上,
 * 单精度下比 t^5 展开稳定得多。
 */
typedef struct {
  float duration;              /* 段时长, 单位 s, 必须 > 0。 */
  float c[EC_MAX_JOINTS][6];   /* 归一化时间下的五次系数, 低次在前。 */
} EC_TrajSeg;

/*
 * 一份完整计划。
 *
 * 时间基准是绝对整数 tick: 第 0 段从 t0_tick 开始, 每 tick 折算
 * tick_period 秒。用整数 tick 而不是累加浮点时间, 是为了让"现在是轨迹上的
 * 哪一点"只取决于当前时刻, 不取决于被调用过几次——后者正是逐周期插值器
 * 里时间会随调用次数漂移的根源。
 *
 * t 超过 total_duration 后一直返回末端点且 dq/ddq 为 0, 即自动进入保持,
 * 不需要外部显式切状态, 消费者也永远不会取到无效值。
 */
typedef struct {
  uint32_t seq;            /* 发布序号, 每次提交递增, 供消费者识别换计划。 */
  uint32_t t0_tick;        /* 第 0 段起点对应的绝对 tick。 */
  float tick_period;       /* 一个 tick 的秒数, 工程里是 0.001f。 */
  float total_duration;    /* 各段时长之和, 单位 s。 */
  uint8_t joint_count;     /* 有效关节数。 */
  uint8_t seg_count;       /* 有效段数; 0 表示纯保持计划。 */
  uint8_t status;          /* EC_TrajStatus。 */
  uint8_t source_id;       /* 命令来源标识, 库不解释, 供上层仲裁和诊断用。 */
  float q_hold[EC_MAX_JOINTS];      /* 轨迹走完后的保持点, 单位 rad。 */
  EC_TrajSeg seg[EC_TRAJ_MAX_SEG];
} EC_TrajPlan;

/*
 * 构造计划时使用的约束。只在构造时读, 求值路径不碰。
 *
 * min_duration / max_duration 是段时长的上下界: 命令给的时长若导致
 * 速度或加速度越限, 构造函数会把时长按比例拉长再重新拟合, 拉到
 * max_duration 仍不可行才报 EC_TRAJ_ERR_INFEASIBLE。
 */
typedef struct {
  uint8_t joint_count;
  float q_min[EC_MAX_JOINTS];    /* rad */
  float q_max[EC_MAX_JOINTS];    /* rad */
  float dq_max[EC_MAX_JOINTS];   /* rad/s, 取正值 */
  float ddq_max[EC_MAX_JOINTS];  /* rad/s^2, 取正值 */
  float min_duration;            /* s */
  float max_duration;            /* s */
} EC_TrajLimits;

/* 用默认值填充约束: 限位取 +-EC_PI, 速度 1 rad/s, 加速度 10 rad/s^2。
 * 调用后按需覆盖单项。joint_count 越界返回 false。 */
bool ec_traj_limits_init(EC_TrajLimits *limits, uint8_t joint_count);

/* ------------------------------- 求值 ------------------------------- */

/*
 * 在相对时刻 t (单位 s, 相对 t0_tick) 求值。
 *
 * t < 0 按 t = 0 处理, t > total_duration 按保持处理; 两种情况都返回 true,
 * 因为"问了一个超出范围的时刻"对消费者不是错误——MPC 预测时域跨过轨迹末尾
 * 是常态, 此时正确答案就是停在终点。
 *
 * out->joint_count 由本函数写入。plan 不会被修改。
 */
bool ec_traj_eval_time(const EC_TrajPlan *plan, float t,
                       EC_JointStateReference *out);

/*
 * 在绝对 tick 求值。内部按 (int32_t)(tick - plan->t0_tick) 计算相对 tick,
 * 因此 tick 计数器回绕不影响结果, 只要读写间隔远小于 2^31 个 tick。
 */
bool ec_traj_eval_tick(const EC_TrajPlan *plan, uint32_t tick,
                       EC_JointStateReference *out);

/*
 * 只求单个关节。MPC 采用交错调度, 一个 tick 只解一个关节的 QP, 只需要
 * 该关节在时域上的 N 个参考点, 用这个接口可以省掉其余 5 个关节的计算。
 * q / dq / ddq 允许为 NULL, 表示不需要该项。
 */
bool ec_traj_eval_joint_time(const EC_TrajPlan *plan, float t, uint8_t joint,
                             float *q, float *dq, float *ddq);

/* ------------------------------- 构造 ------------------------------- */

/* 构造一份纯保持计划: 停在 q_hold, 无分段。上电初始化和故障兜底都用它。 */
bool ec_traj_plan_hold(EC_TrajPlan *plan, uint8_t joint_count,
                       const float *q_hold, uint32_t t0_tick,
                       float tick_period, uint8_t status, uint8_t source_id);

/*
 * 单段点到点: 从 start 描述的当前参考状态 (q, dq, ddq) 平滑过渡到
 * q_target, 终点速度加速度为 0, 标称时长 duration。
 *
 * 起点带上 dq/ddq 而不是只带 q, 是为了在上一条轨迹还没走完时接新命令能
 * 保持 C2 连续: 新段的起点边界条件直接取当前参考点的一阶二阶导, 拼接处
 * 位置、速度、加速度都连续, 不会在换命令的瞬间给电机一个加速度台阶。
 *
 * duration 会在必要时被拉长以满足约束; 实际采用的时长通过 applied_duration
 * 返回 (可为 NULL)。
 */
EC_TrajResult ec_traj_plan_point_to_point(
    EC_TrajPlan *plan, const EC_TrajLimits *limits,
    const EC_JointStateReference *start, const float *q_target, float duration,
    uint32_t t0_tick, float tick_period, uint8_t source_id,
    float *applied_duration);

/*
 * 多节点: 依次经过 nodes[0..node_count-1], 第 i 段时长 durations[i]。
 * 中间节点的速度用无量纲中心差分估计 (相邻节点方向一致时取平均速度,
 * 反向时取 0), 加速度取 0, 以免在节点处停顿; 末节点速度加速度为 0。
 */
EC_TrajResult ec_traj_plan_multi_point(
    EC_TrajPlan *plan, const EC_TrajLimits *limits,
    const EC_JointStateReference *start,
    const float nodes[][EC_MAX_JOINTS], const float *durations,
    uint8_t node_count, uint32_t t0_tick, float tick_period,
    uint8_t source_id);

/*
 * 刹车: 从 start 的当前状态按加速度上限减速到停, 生成一段计划,
 * status 置 EC_TRAJ_BRAKING。遥控器掉线、模式切换、上层报错都走这里,
 * 而不是直接把参考值跳回某个固定点。
 */
EC_TrajResult ec_traj_plan_brake(EC_TrajPlan *plan,
                                 const EC_TrajLimits *limits,
                                 const EC_JointStateReference *start,
                                 uint32_t t0_tick, float tick_period,
                                 uint8_t source_id);

/* ------------------------------- 发布 ------------------------------- */

/*
 * 双缓冲发布通道: 单写者、多读者。
 *
 * 写者拿 begin_write() 得到的备用槽慢慢填, 填完 commit() 翻转索引;
 * 读者任何时刻 read() 到的都是一份完整计划, 不会看到半份。索引是单个
 * 对齐的 uint32, 在 Cortex-M 上单次读写本身就是原子的, commit 前后各插
 * 一道屏障保证载荷写入不会被编译器挪到索引翻转之后。
 *
 * 约束: 读者必须在两次 commit 之间完成对所指计划的使用。工程里读者是
 * 2 ms 的 GimbalTask 和 1 ms 的 MPC tick, 单次使用只有几微秒, 而命令
 * 最快也 14 ms 才来一次, 两个槽足够。若以后出现更慢的读者, 改成三槽或
 * 在读侧用 seq 做一次前后比对即可。
 */
typedef struct {
  EC_TrajPlan slot[2];
  volatile uint32_t active; /* 当前可读槽的索引, 0 或 1。 */
  uint32_t next_seq;        /* 下一次 commit 使用的序号。 */
} EC_TrajChannel;

/* 初始化通道, 两个槽都置为停在 q_hold 的保持计划。 */
bool ec_traj_channel_init(EC_TrajChannel *channel, uint8_t joint_count,
                          const float *q_hold, uint32_t t0_tick,
                          float tick_period);

/* 取备用槽用于填写; 返回的指针在 commit() 之前有效。 */
EC_TrajPlan *ec_traj_channel_begin_write(EC_TrajChannel *channel);

/* 提交备用槽, 使其成为当前可读计划, 并写入递增的 seq。 */
void ec_traj_channel_commit(EC_TrajChannel *channel);

/* 取当前可读计划; 返回的指针由通道持有。 */
const EC_TrajPlan *ec_traj_channel_read(const EC_TrajChannel *channel);

#endif /* ENGINEER_COMPONENTS_C_TRAJ_H */
