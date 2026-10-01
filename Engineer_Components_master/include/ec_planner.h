#ifndef ENGINEER_COMPONENTS_C_PLANNER_H
#define ENGINEER_COMPONENTS_C_PLANNER_H

#include "ec_transform.h"

/*
 * 轨迹规划器：标量线性、关节空间和笛卡尔空间。
 *
 * 三种规划器：
 *   - EC_LinearPlanner      标量线性规划，在两个标量之间做线性插值
 *   - EC_JointPlanner       关节空间规划，各关节独立做五次多项式插值
 *   - EC_CartesianPlanner   笛卡尔空间规划，直线或圆弧位置插值 + 四元数 SLERP 姿态插值
 *
 * 共同使用模式：
 *   1. init()  初始化规划器，设置控制周期
 *   2. set()   设置起止点和规划总时间
 *   3. 循环调用 plan()，每次推进一个控制周期，获取当前插值结果
 *   4. finished() 判断是否到达终点
 *
 * 所有规划器不使用动态内存，适合嵌入式环境。
 * 时间单位为秒（s），角度单位为弧度（rad），长度单位由应用统一。
 */

#ifdef __cplusplus
extern "C" {
#endif

/* ── 标量线性规划器 ── */
/* 在起止标量之间做线性插值，每个控制周期推进一次。 */
/* 插值公式：out = start + (end - start) × (elapsed / plan_time)。 */

/* 初始化标量规划器。
 * ctrl_period: 控制周期（s）；若 <= 0，内部自动采用 0.001s。 */
void ec_linear_planner_init(EC_LinearPlanner *planner, float ctrl_period);

/* 设置规划起止点和总时间。
 * start, end: 起止标量，物理单位由应用统一（如 rad、m 等）。
 * plan_time:  规划总时间（s），若 <= 0 则立即完成。 */
void ec_linear_planner_set(EC_LinearPlanner *planner, float start, float end,
                           float plan_time);

/* 执行一个控制周期的规划，返回当前标量位置。
 * 每调用一次 elapsed 增加 ctrl_period，到达 plan_time 后 finished 置 true。 */
float ec_linear_planner_plan(EC_LinearPlanner *planner);

/* 查询规划是否已完成。 */
bool ec_linear_planner_finished(const EC_LinearPlanner *planner);

/* ── 笛卡尔空间规划器 ── */
/* 在两个笛卡尔位姿之间做直线或圆弧插值。 */
/* 直线位置使用五次多项式（quintic）：s(t) = 10t³ - 15t⁴ + 6t⁵， */
/* 保证初末速度和加速度为零；圆弧角度按时间线性推进，即角速度恒定。 */
/* 姿态插值使用四元数球面线性插值（SLERP）。 */

/* 初始化笛卡尔规划器。
 * ctrl_period: 控制周期（s）；若 <= 0，内部自动采用 0.001s。 */
void ec_cartesian_planner_init(EC_CartesianPlanner *planner, float ctrl_period);

/* 设置直线规划：位置沿空间直线插值，姿态做四元数 SLERP。
 * start, end: 起止笛卡尔位姿；位置长度单位由应用统一，姿态四元数 [w,x,y,z]。
 * plan_time:  规划总时间（s），若 <= 0 则立即完成。 */
void ec_cartesian_planner_set_linear(EC_CartesianPlanner *planner,
                                     EC_CartesianCmd start,
                                     EC_CartesianCmd end, float plan_time);

/* 设置圆弧规划：位置沿起点-圆心-终点确定的圆弧插值，姿态做四元数 SLERP。
 * start, end: 起止笛卡尔位姿。
 * center:     圆弧圆心位置，长度单位由应用统一。
 * plan_time:  规划总时间（s）。
 * major_arc:  false 选择不超过 180° 的路径，true 选择互补方向的路径；
 *             起点和终点恰好对径时两条路径均为 180°，但仅凭三点无法确定圆面。
 * 若半径为零、两条半径共线或起终点到圆心不等距，规划器退化为直线插值。 */
void ec_cartesian_planner_set_circle(EC_CartesianPlanner *planner,
                                     EC_CartesianCmd start,
                                     EC_CartesianCmd end, EC_Vec3 center,
                                     float plan_time, bool major_arc);

/* 执行一个控制周期的规划，返回当前笛卡尔位姿。
 * 位置：直线使用五次多项式调速，圆弧按恒定角速度推进。
 * 姿态：四元数 SLERP，参数 u = elapsed / plan_time 线性推进。
 * 每调用一次 elapsed 增加 ctrl_period，到达 plan_time 后 finished 置 true。 */
EC_CartesianCmd ec_cartesian_planner_plan(EC_CartesianPlanner *planner);

/* 查询规划是否已完成。 */
bool ec_cartesian_planner_finished(const EC_CartesianPlanner *planner);

/* ── 关节空间规划器 ── */
/* 各关节独立做五次多项式插值，保证初末速度和加速度为零；
 * 加加速度（jerk）在端点通常不连续。 */
/* 插值公式：q_i(t) = start_i + (end_i - start_i) × quintic(u)，u = elapsed / plan_time。 */

/* 初始化关节规划器。
 * ctrl_period: 控制周期（s）；若 <= 0，内部自动采用 0.001s。
 * joint_count: 参与规划的关节数，不超过 EC_MAX_JOINTS。 */
void ec_joint_planner_init(EC_JointPlanner *planner, float ctrl_period,
                           uint8_t joint_count);

/* 设置第 index 个关节是否按最短周期角差规划。
 * periodic = true  时，若角差 > π 则绕另一方向走（适用于连续旋转关节）。
 * periodic = false 时，直接从 start 走到 end，不绕最短路径（默认）。
 * index 从 0 开始。 */
void ec_joint_planner_set_periodic(EC_JointPlanner *planner, uint8_t index,
                                   bool periodic);

/* 设置规划起止关节角和总时间。
 * start, end: 起止关节角数组，长度至少为 joint_count，单位 rad。
 *             对于 periodic 关节，end 会被自动调整到最短周期角差。
 * plan_time:  规划总时间（s），若 <= 0 则立即完成。
 * 成功返回 true，空指针或 plan_time < 0 返回 false。 */
bool ec_joint_planner_set(EC_JointPlanner *planner, const float *start,
                          const float *end, float plan_time);

/* 执行一个控制周期的规划，输出当前关节角到 out。
 * out:      输出数组，至少能容纳 joint_count 个 float，单位 rad。
 * out_count: out 数组的元素容量，至少为 joint_count。
 * 每调用一次 elapsed 增加 ctrl_period，到达 plan_time 后 finished 置 true。 */
void ec_joint_planner_plan(EC_JointPlanner *planner, float *out,
                           size_t out_count);

/* 执行一个控制周期并输出位置、速度和加速度。
 * position 单位 rad，velocity 单位 rad/s，acceleration 单位 rad/s^2。
 * position 必须有效；velocity 和 acceleration 可传 NULL，此时仅输出位置。
 * 所有非 NULL 输出数组均至少包含 joint_count 个元素。 */
void ec_joint_planner_plan_state(EC_JointPlanner *planner, float *position,
                                 float *velocity, float *acceleration,
                                 size_t out_count);

/* 查询规划是否已完成。 */
bool ec_joint_planner_finished(const EC_JointPlanner *planner);

#ifdef __cplusplus
}
#endif

#endif
