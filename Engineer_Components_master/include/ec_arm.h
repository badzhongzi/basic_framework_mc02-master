#ifndef ENGINEER_COMPONENTS_C_ARM_H
#define ENGINEER_COMPONENTS_C_ARM_H

#include "ec_ikine.h"
#include "ec_link.h"
#include "ec_planner.h"

/*
 * 通用机械臂状态机：整合反馈更新、命令接收、轨迹规划、逆解选解和参考角生成。
 *
 * EC_Arm 是框架的最高层接口，一条调用链即可完成完整的控制周期：
 *
 *   典型控制循环（每个周期执行一次）：
 *     1. ec_arm_set_feedback()   从编码器读取当前关节角，写入反馈
 *     2. ec_arm_update()         基于反馈角计算正运动学，刷新末端位姿
 *     3. ec_arm_set_command()    接收新的运动命令（仅在需要时调用）
 *     4. ec_arm_generate_reference()  执行规划/逆解/选解，生成关节参考角
 *     5. ec_arm_reference()      读取参考角，发送给伺服驱动器
 *
 * 支持四种规划模式：
 *   - EC_PLAN_JOINT:      关节空间插值，各关节独立做五次多项式插值
 *   - EC_PLAN_LINEAR:     笛卡尔直线插值 + 逆解
 *   - EC_PLAN_CIRCULAR:   笛卡尔圆弧插值 + 逆解
 *   - EC_PLAN_NONE:       不插值，直接使用关节命令或执行一次逆解
 *
 * 逆解通过 ec_solve_ikine_fn 回调注入，框架不绑定特定构型。
 * 多解选择策略：在所有候选解中，选择满足关节限位且离当前反馈角最近的那组。
 *
 * 所有对象使用调用方提供的静态存储，不进行堆分配。
 * 角度单位为 rad，长度单位由应用统一。
 */

#ifdef __cplusplus
extern "C" {
#endif

/* ── 逆解回调类型 ── */

/* 自定义逆解回调函数签名。
 *
 * cmd:         目标笛卡尔位姿（位置 + 四元数姿态）。
 * free_joints: 冗余关节参数，单位 rad。EC_PLAN_JOINT 的笛卡尔节点会传入
 *              该节点的 joint 数组；普通笛卡尔轨迹和 EC_PLAN_NONE 传入 NULL。
 *              候选解与当前关节角的距离由框架计算，与此参数无关。
 * joint_count: 关节数量。
 * solution:    输出候选解数组，EC_MAX_IK_SOLUTIONS 行 × EC_MAX_JOINTS 列，
 *              每行一组完整的关节角候选解，单位 rad。
 * user:        用户数据指针，原样传回，便于绑定具体机械臂参数（如连杆长度数组）。
 *
 * 返回值: 有效候选解的数量（0 表示无解）。
 *
 * 示例：包装 6 自由度的 ec_ikine1 作为回调（需使用中间数组适配维度）
 *   uint8_t my_ikine(
 *                    EC_ModuleCartesianCmd cmd,
 *                    const float *free_joints,
 *                    uint8_t joint_count,
 *                    float solution[EC_MAX_IK_SOLUTIONS][EC_MAX_JOINTS],
 *                    void *user)
 * {
 *       const float *len = (const float *)user;
 *       float raw[8][6];
 *       uint8_t n, i, j;
 *       if (joint_count != 6 || !len) return 0;
 *       n = ec_ikine1(cmd, len, raw);
 *       if (n > EC_MAX_IK_SOLUTIONS) n = EC_MAX_IK_SOLUTIONS;
 *       for (i = 0; i < n; ++i)
 *         for (j = 0; j < 6; ++j) solution[i][j] = raw[i][j];
 *       return n;
 *   }
 */
typedef uint8_t (*ec_solve_ikine_fn)(EC_ModuleCartesianCmd cmd,
                                     const float *free_joints,
                                     uint8_t joint_count,
                                     float solution[EC_MAX_IK_SOLUTIONS][EC_MAX_JOINTS],
                                     void *user);

/* ── 机械臂状态对象 ── */

/* 通用机械臂状态对象，整合连杆模型、逆解回调、反馈/参考角、位姿和规划器。 */
typedef struct {

  /* ── 身份 ── */
  uint8_t joint_count;                                     /* 实际关节数 */
  EC_SerialLink *serial;                                   /* 串联连杆模型指针（调用方持有） */
  ec_solve_ikine_fn solve_ikine;                      /* 自定义逆解回调；仅笛卡尔规划时使用，关节规划时可设 NULL。 */
  void *user;                                                  /* 逆解回调的用户数据，原样传回 solve_ikine。 */

  /* ── 反馈 ── */
  float joint_feedback[EC_MAX_JOINTS];       /* 当前反馈角度（来自编码器），单位 rad。 */
  float joint_velocity_feedback[EC_MAX_JOINTS];
                                                                   /* 当前反馈角速度，单位 rad/s。
                                                                    * 由 ec_arm_set_velocity_feedback() 写入，本对象内部不使用，
                                                                    * 存在的意义是给需要状态反馈的控制器（如 MPC 的误差锚点
                                                                    * x = [q, dq]）一个统一的取值处，不必各自再开一份缓存。
                                                                    * 没有数据源时保持为 0。 */

  /* ── 参考 ── */
  float joint_reference[EC_MAX_JOINTS];                      /* 当前目标角度（输出给伺服），单位 rad。 */
  float joint_velocity_reference[EC_MAX_JOINTS];         /* 当前目标角速度，单位 rad/s。 */                                                  
  float joint_acceleration_reference[EC_MAX_JOINTS];  /* 当前目标角加速度，单位 rad/s^2。 */                                         
  EC_JointStateReference state_reference;                    /* 完整状态参考视图；前 joint_count 项有效。(是前三个的打包, 有点冗余了, 但是方便对接某些函数的签名) */

  /* ── 上一拍参考（笛卡尔轨迹差分用） ── */
  float previous_reference[EC_MAX_JOINTS];                      /* 上一个成功输出的关节位置，单位 rad；用于笛卡尔轨迹差分。 */
  float previous_velocity_reference[EC_MAX_JOINTS];         /* 上一个输出的关节速度，单位 rad/s；用于笛卡尔轨迹差分。 */
  bool state_history_valid;                                                  /* 是否已有可用于差分的上一采样点。 */

  /* ── 限位 ── */
  float joint_min[EC_MAX_JOINTS];                                      /* 关节软件下限位，单位 rad；从 serial->links[i].mdh.qmin 初始化。 */
  float joint_max[EC_MAX_JOINTS];                                      /* 关节软件上限位，单位 rad；从 serial->links[i].mdh.qmax 初始化。 */

  /* ── 位姿 ── */
  EC_ArmPose pose;                                                            /* 当前机械臂位姿；由 ec_arm_update() 刷新。 */

  /* ── 旧规划器planner（现正在转向traj, 保留接口, 可正常使用, 但不能同时和traj使用） ── */
  EC_ArmFsmCmd command;                                              /* 当前 FSM 命令；由 ec_arm_set_command() 设置。 */
  EC_JointPlanner joint_planner;                                          /* 关节空间规划器；EC_PLAN_JOINT 时使用。 */
  EC_CartesianPlanner cartesian_planner;                            /* 笛卡尔空间规划器；EC_PLAN_LINEAR/CIRCULAR 时使用。 */
  bool planner_active;                                                         /* 当前是否正在执行一个尚未完成的规划段。 */
  uint8_t planner_segment_index;                                       /* 关节规划时为下一个目标节点索引（等于 node_count 表示完成）；
                                                                                           *笛卡尔规划时 0/1 表示执行中/完成。 */

  bool initialized;                                                            /* 是否已完成 ec_arm_init() 初始化。 */
} EC_Arm;

/* ── 初始化 ── */

/* 初始化机械臂对象。
 * arm:         待初始化的机械臂对象。
 * joint_count: 实际关节数，必须 > 0 且 <= EC_MAX_JOINTS。
 * serial:      串联连杆模型，必须已追加至少 joint_count 个连杆。
 * solve_ikine: 逆解回调函数；仅笛卡尔规划时需要，关节规划时可传 NULL。
 * user:        传回 solve_ikine 的用户数据指针；solve_ikine 为 NULL 时忽略。
 * 初始化时内部的控制周期设置为 0.001 s；需要修改时调用 ec_arm_set_control_period()。
 * 成功返回 true；参数非法或 serial 连杆数不足时返回 false。 */
bool ec_arm_init(EC_Arm *arm, uint8_t joint_count, EC_SerialLink *serial,
                 ec_solve_ikine_fn solve_ikine, void *user);

/* 设置 EC_Arm 内部关节/笛卡尔规划器的控制周期。
 * control_period: 实际调用 ec_arm_generate_reference() 的周期，单位 s，必须为有限正数。
 * ec_arm_init() 默认使用 0.001 s；非 1 kHz 控制循环应在初始化后调用本函数。
 * 建议仅在尚未开始一条轨迹时修改。成功返回 true，参数非法时返回 false。 */
bool ec_arm_set_control_period(EC_Arm *arm, float control_period);

/* ── 反馈与正解 ── */

/* 设置反馈关节角（来自编码器），单位 rad。
 * 仅更新 joint_feedback 数组，不触发正解计算；
 * 需随后调用 ec_arm_update() 刷新末端位姿。 */
void ec_arm_set_feedback(EC_Arm *arm, const float *joint);

/* 设置反馈关节角速度，单位 rad/s。
 * 只写 joint_velocity_feedback 数组，不参与正解也不影响任何现有逻辑。 */
void ec_arm_set_velocity_feedback(EC_Arm *arm, const float *joint_velocity);

/* 直接写入一份完整的状态参考（q / q_dot / q_ddot）。
 *
 * 给可求值轨迹层（ec_traj.h）用：那一层自己按绝对时间求值，算出的参考点
 * 通过这个接口灌进 EC_Arm，下游读 ec_arm_state_reference() 的代码不用改。
 *
 * 与 ec_arm_generate_reference() 互斥使用——两者都写同一组参考字段，
 * 同一个控制周期里只应调用其中一个。本函数不碰 command、不碰规划器状态、
 * 不做限幅也不做角度归一化：可行性和限位已经在构造轨迹时检查过，在输出级
 * 再截断会让同一时刻求值两次得到不同结果。
 *
 * state->joint_count 必须等于 arm->joint_count。成功返回 true。 */
bool ec_arm_set_state_reference(EC_Arm *arm,
                                const EC_JointStateReference *state);

/* 基于当前 joint_feedback 执行正运动学，刷新 pose。
 * 内部调用 ec_serial_link_fkine() 计算末端齐次变换，
 * 并提取位置和四元数姿态写入 pose.cartesian。 */
void ec_arm_update(EC_Arm *arm);

/* ── 命令与规划 ── */

/* 设置 FSM 运动命令，并将路径推进状态重置到第一段。
 * command: 包含路径节点、规划类型和各段时间的运动描述。
 * EC_PLAN_JOINT 至少需要 1 个目标节点，EC_PLAN_LINEAR 至少需要 2 个笛卡尔节点，
 * EC_PLAN_CIRCULAR 至少需要 3 个笛卡尔节点，EC_PLAN_NONE 使用 nodes[0]。
 * 关节节点的 joint_count 必须等于 arm->joint_count；所有实际使用的轨迹时间
 * 必须为有限非负数。笛卡尔节点还要求已经注册 solve_ikine 回调，位姿分量
 * 均为有限值且四元数非零。圆弧的起点和终点必须与圆心等距且不共线。
 * 成功返回 true；命令不满足上述条件时返回 false，且保留原命令不变。
 * 设置命令后，需在控制循环中反复调用 ec_arm_generate_reference() 执行规划。 */
bool ec_arm_set_command(EC_Arm *arm, const EC_ArmFsmCmd *command);

/* 执行一个控制周期的规划，生成关节参考角。
 *
 * 根据 command.plan_type 分支处理：
 *   EC_PLAN_JOINT:
 *     joint_feedback 是隐式起点，依次执行 nodes[0] 到 nodes[node_count-1]；
 *     第 i 段从前一节点运动到 nodes[i]，使用 traj_time[i]。关节节点直接作为
 *     该段终点；笛卡尔节点先调用 solve_ikine，再选择离上一段终点最近且满足
 *     限位的候选关节解。
 *   EC_PLAN_LINEAR:
 *     nodes[0] 是起始笛卡尔位姿，nodes[1] 是终止位姿，使用 traj_time[0]；
 *     每周期对插值位姿求逆解。该模式只使用 nodes[0]、nodes[1] 和 traj_time[0]；
 *     nodes[2] 及以后节点不参与规划。
 *   EC_PLAN_CIRCULAR:
 *     nodes[0] 是起始笛卡尔位姿，nodes[1] 是终止位姿，
 *     nodes[2].cartesian.pos 是圆心，使用 traj_time[0] 并默认选择短弧；
 *     该模式只使用前三个节点，nodes[3] 及以后节点不参与规划。
 *   EC_PLAN_NONE:
 *     若命令为笛卡尔空间则直接逆解，关节空间则直接使用命令中的关节角。
 *
 * 成功返回 true；未初始化或无命令时返回 false。
 * 笛卡尔规划时若逆解无有效解（不满足限位），返回 false。
 * 规划完成后继续调用会保持最后一组关节参考状态并返回 true。
 * 生成结果同时写入 q、q_dot、q_ddot，可通过 ec_arm_state_reference() 读取。 */
bool ec_arm_generate_reference(EC_Arm *arm);

/* ── 结果读取 ── */

/* 获取当前位姿的只读指针，由 EC_Arm 持有，无需调用方释放。
 * 包含笛卡尔位姿、关节角和末端齐次变换三种表示。
 * arm 为 NULL 时返回 NULL。 */
const EC_ArmPose *ec_arm_pose(const EC_Arm *arm);

/* 获取当前关节参考角的只读指针，由 EC_Arm 持有，无需调用方释放。
 * 数组长度为 arm->joint_count，单位 rad。
 * arm 为 NULL 时返回 NULL。 */
const float *ec_arm_reference(const EC_Arm *arm);

/* 获取当前完整关节状态参考；返回的指针由 EC_Arm 持有，无需释放。
 * q 单位 rad，q_dot 单位 rad/s，q_ddot 单位 rad/s^2。 */
const EC_JointStateReference *ec_arm_state_reference(const EC_Arm *arm);

#ifdef __cplusplus
}
#endif

#endif
