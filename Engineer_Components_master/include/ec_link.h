#ifndef ENGINEER_COMPONENTS_C_LINK_H
#define ENGINEER_COMPONENTS_C_LINK_H

#include "ec_transform.h"

/*
 * MDH 连杆建模、串联机械臂正运动学、雅可比和重力补偿接口。
 *
 * 本模块基于 MDH（Modified DH，Craig）约定建立连杆运动学模型，提供：
 *   - 单节连杆的 MDH 参数初始化与正/逆变换计算
 *   - 串联连杆模型的构建（逐节追加）与正运动学求解
 *   - 末端位姿的多种表示提取（齐次变换、旋转矩阵、位置、四元数）
 *   - 运行时修改 MDH 参数、关节限位和动力学参数
 *   - 几何雅可比矩阵计算
 *   - 重力补偿力矩计算
 *
 * 所有对象使用调用方提供的静态存储，不进行堆分配，适合嵌入式环境。
 * 角度单位为 rad，长度单位由应用统一（示例使用 m）。
 */

#ifdef __cplusplus
extern "C" {
#endif

/* ── 单节连杆 ── */

/* 初始化 MDH 参数。
 * offset: 关节零位偏置（rad），实际变换角为 q + offset。
 * d, a:   MDH 长度参数，单位由应用统一（示例使用 m）。
 * alpha:  MDH 扭转角（rad）。
 * qmin, qmax: 关节软件限位（rad）。 */
void ec_mdh_init(EC_MDH *mdh, float offset, float d, float a, float alpha,
                float qmin, float qmax);

/* 计算单节连杆的正变换矩阵：T = Rot(x,α)·Trans(x,a)·Rot(z,θ+offset)·Trans(z,d)。
 * q: 关节角（rad），函数内部会加上 offset。
 * 返回 4×4 齐次变换矩阵。 */
EC_Mat4 ec_mdh_forward(const EC_MDH *mdh, float q);

/* 计算单节连杆正变换的严格刚体逆变换：
 * T⁻¹ = [Rᵀ, -Rᵀp; 0, 1]。q 的单位为 rad，平移单位同 MDH 参数。 */
EC_Mat4 ec_mdh_backward(const EC_MDH *mdh, float q);

/* 初始化连杆对象：复制 MDH 参数，质量、质心和惯量矩阵均置零。
 * 后续可通过 ec_link_set_dynamic() 设置动力学参数。 */
void ec_link_init(EC_Link *link, const EC_MDH *mdh);

/* 设置连杆动力学参数，用于重力补偿和动力学计算。
 * mass:     质量（kg）。
 * centroid: 质心在连杆坐标系中的位置，长度单位同 MDH。
 * inertia:  关于质心的惯量矩阵（kg·m²，当长度单位为 m 时）。 */
void ec_link_set_dynamic(EC_Link *link, float mass, EC_Vec3 centroid,
                         EC_Mat3 inertia);

/* ── 串联连杆模型 ── */

/* 初始化串联连杆对象：连杆数清零，正解结果设为单位变换。 */
void ec_serial_link_init(EC_SerialLink *serial);

/* 向串联模型末尾追加一节连杆；最多追加到 EC_MAX_JOINTS 节。
 * 成功返回 true，超过上限时返回 false。 */
bool ec_serial_link_append(EC_SerialLink *serial, const EC_Link *link);

/* ── 正运动学 ── */

/* 计算串联正运动学：逐节累乘各连杆的正变换，得到末端齐次变换。
 * theta: 关节角数组，长度至少为 serial->count，单位 rad。
 * 结果缓存在 serial->fkine 和 serial->transforms[] 中。
 * 成功返回 true，空指针或 theta 为 NULL 时返回 false。 */
bool ec_serial_link_fkine(EC_SerialLink *serial, const float *theta);

/* 获取最近一次正解的末端 4×4 齐次变换矩阵。 */
EC_Mat4 ec_serial_link_get_t(const EC_SerialLink *serial);

/* 获取最近一次正解的末端 3×3 旋转矩阵。 */
EC_Mat3 ec_serial_link_get_r(const EC_SerialLink *serial);

/* 获取最近一次正解的末端位置向量；长度单位同 MDH。 */
EC_Vec3 ec_serial_link_get_p(const EC_SerialLink *serial);

/* 获取最近一次正解的末端姿态四元数 [w, x, y, z]。 */
EC_Quat ec_serial_link_get_quat(const EC_SerialLink *serial);

/* ── 运行时参数修改 ── */

/* 修改第 index 节连杆的 MDH 参数（保留原有 qmin/qmax 限位不变）。
 * index 从 0 开始，成功返回 true，越界返回 false。 */
bool ec_serial_link_set_mdh(EC_SerialLink *serial, uint8_t index,
                            float offset, float d, float a, float alpha);

/* 读取第 index 节连杆的 MDH 参数。
 * 成功返回 true，越界或 mdh 为 NULL 时返回 false。 */
bool ec_serial_link_get_mdh(const EC_SerialLink *serial, uint8_t index,
                            EC_MDH *mdh);

/* 修改第 index 节连杆的关节软件限位；qmin、qmax 单位均为 rad。
 * 要求 qmin <= qmax，成功返回 true，越界或参数非法返回 false。 */
bool ec_serial_link_set_limit(EC_SerialLink *serial, uint8_t index,
                               float qmin, float qmax);

/* 读取第 index 节连杆的关节软件限位；输出单位均为 rad。
 * 成功返回 true，越界或指针为 NULL 时返回 false。 */
bool ec_serial_link_get_limit(const EC_SerialLink *serial, uint8_t index,
                              float *qmin, float *qmax);

/* 修改第 index 节连杆的动力学参数。
 * mass: 质量（kg）；centroid: 质心位置（长度单位同 MDH）；
 * inertia: 惯量矩阵（kg·m²，当长度单位为 m 时）。
 * 成功返回 true，越界返回 false。 */
bool ec_serial_link_set_dynamic(EC_SerialLink *serial, uint8_t index,
                                float mass, EC_Vec3 centroid, EC_Mat3 inertia);

/* 读取第 index 节连杆的动力学参数。
 * 成功返回 true，越界或 dynamic 为 NULL 时返回 false。 */
bool ec_serial_link_get_dynamic(const EC_SerialLink *serial, uint8_t index,
                                EC_DynamicParam *dynamic);

/* ── 雅可比矩阵 ── */

/* 计算几何雅可比矩阵，输出按行优先存储为 6 行 × N 列（N = serial->count）。
 * 排列顺序：
 *   行 0~2: 线速度部分 J_v（单位：长度/rad）
 *   行 3~5: 角速度部分 J_ω（单位：rad/rad，工程上通常视为无量纲）
 *   与关节速度相乘后，才得到长度/s 和 rad/s 的末端速度。
 * out_count: out 数组的元素容量，至少为 6 × N。
 * 成功返回 true，容量不足或参数非法返回 false。
 * 注意：调用前必须先调用 ec_serial_link_fkine() 计算正解。 */
bool ec_serial_link_jacobian(const EC_SerialLink *serial, float *out,
                             size_t out_count);

/* ── 重力补偿算法选择（编译期切换） ── */

/* EC_GRAVITY_JACOBIAN_TRANSPOSE (0, 默认):
 *   雅可比转置法（虚功法），O(N²)
 *   公式: τ_i = Σ_{k=i}^{N-1} z_i · (r_{i→c_k} × F_k)
 *   F_k 使用 +Z 方向的等效支撑力，输出为抵消 -Z 重力的补偿力矩。
 *   优点: 实现简单，复用正运动学
 *   适合: N ≤ 6 的常见机械臂
 *
 * EC_GRAVITY_NEWTON_EULER (1):
 *   牛顿欧拉递推法（力/力矩传播），O(N)
 *   在世界坐标系中从末端向基座递推累积力和力矩，再投影到关节轴。
 *   优点: 线性复杂度，N 较大时更快
 *   适合: N > 6 的冗余机械臂，或对计算时间敏感的场景
 *
 * 两种方法计算同一个重力补偿项；由于浮点运算顺序不同，数值结果可能有
 * 极小的舍入差异。用户可在编译时覆盖: -DEC_GRAVITY_METHOD=1
 */
#define EC_GRAVITY_JACOBIAN_TRANSPOSE  0
#define EC_GRAVITY_NEWTON_EULER        1

#ifndef EC_GRAVITY_METHOD
#define EC_GRAVITY_METHOD  EC_GRAVITY_NEWTON_EULER
#endif

#if (EC_GRAVITY_METHOD != EC_GRAVITY_JACOBIAN_TRANSPOSE) && \
    (EC_GRAVITY_METHOD != EC_GRAVITY_NEWTON_EULER)
#error "EC_GRAVITY_METHOD must be EC_GRAVITY_JACOBIAN_TRANSPOSE (0) or EC_GRAVITY_NEWTON_EULER (1)"
#endif

/* ── 重力补偿 ── */

/* 计算各关节的重力补偿力矩，用于前馈补偿重力项。
 * theta:  关节角数组，长度至少为 serial->count，单位 rad。
 * torque: 输出力矩数组，长度至少为 serial->count，单位 N·m（当长度单位为 m 时）。
 * 重力加速度大小取 9.81 m/s²，物理重力方向沿 -Z 轴；函数输出的是
 * 抵消该重力的补偿力矩，因此内部使用 +Z 方向的等效支撑力。
 * 算法由 EC_GRAVITY_METHOD 宏选择（编译期）：
 *   0 = 雅可比转置法（默认），1 = 牛顿欧拉递推法。
 * 注意：当前实现使用质量和质心计算重力力矩，不使用惯量矩阵；若所有连杆质量
 * 为零则输出为零，单个连杆质量为零也不代表其上游关节的总力矩为零。 */
void ec_serial_link_gravity(const EC_SerialLink *serial, const float *theta,
                            float *torque);

/* 计算串联机械臂的完整逆动力学前馈力矩（牛顿-欧拉递推）。
 * q、q_dot、q_ddot 分别为关节位置（rad）、速度（rad/s）和加速度（rad/s^2），
 * torque 为输出力矩（N·m，长度至少为 serial->count）。
 * 动力学参数来自各连杆的 mass（kg）、centroid（m）和 inertia（kg·m^2）。
 * 计算包含惯性、科氏/离心和重力项；重力方向固定为基座坐标系 -Z，
 * 与 ec_serial_link_gravity() 使用的坐标约定一致。所有输入数组必须有效。 */
bool ec_serial_link_inverse_dynamics(const EC_SerialLink *serial,
                                     const float *q, const float *q_dot,
                                     const float *q_ddot, float *torque);

#ifdef __cplusplus
}
#endif

#endif
