#ifndef ENGINEER_COMPONENTS_C_OPERATION_H
#define ENGINEER_COMPONENTS_C_OPERATION_H

#include "ec_math.h"

/*
 * 对常用向量、四元数和齐次变换操作的语义化封装。
 *
 * 本模块是对 ec_math.h 中底层运算的语义化别名，提供更直观的函数名：
 *   - ec_cross()       → ec_vec3_cross()    三维向量叉乘
 *   - ec_dot()         → ec_vec3_dot()      三维向量点乘
 *   - ec_quat_cross()  → ec_quat_mul()      四元数 Hamilton 乘法
 *   - ec_htm_multiply()→ ec_mat4_mul()      齐次变换矩阵乘法
 *
 * 齐次变换乘法 ec_htm_multiply() 按列向量约定执行：
 *   结果为 a·b；若将其作用于向量，实际先应用 b，再应用 a。
 *   若 a、b 分别是连续坐标系变换，具体 A→B/B→C 的命名需与应用的
 *   坐标变换方向约定保持一致；矩阵公式见下方说明。
 *
 * 这些函数与底层实现完全等价，仅提供更易读的调用接口，
 * 在代码语义明确时优先使用本模块的函数名。
 */

#ifdef __cplusplus
extern "C" {
#endif

/* ── 向量运算 ── */

/* 三维向量叉乘：a × b。
 * 结果向量垂直于 a 和 b 所在平面，方向由右手定则确定。
 * 模长 ‖a × b‖ = ‖a‖·‖b‖·sin(θ)，θ 为两向量夹角。 */
EC_Vec3 ec_cross(EC_Vec3 a, EC_Vec3 b);

/* 三维向量点乘：a · b = a.x*b.x + a.y*b.y + a.z*b.z。
 * 结果为标量，a · b = ‖a‖·‖b‖·cos(θ)。
 * 零结果表示两向量正交。 */
float ec_dot(EC_Vec3 a, EC_Vec3 b);

/* ── 四元数运算 ── */

/* 四元数 Hamilton 乘法：a ⊗ b。
 * 表示先应用 b 的旋转，再应用 a 的旋转（与矩阵乘法顺序一致）。
 * 即若 q1 表示 R1，q2 表示 R2，则 ec_quat_cross(q1, q2) 表示 R1·R2。 */
EC_Quat ec_quat_cross(EC_Quat a, EC_Quat b);

/* ── 齐次变换运算 ── */

/* 齐次变换矩阵乘法：T = a × b。
 * 等价于 ec_mat4_mul(a, b)，但语义更明确——组合两个刚体变换。
 * 若 a = |R1 p1|，b = |R2 p2|，则结果为 |R1·R2  R1·p2+p1|。
 *                              |0  1|     |0  1|           |0        1| */
EC_Mat4 ec_htm_multiply(EC_Mat4 a, EC_Mat4 b);

#ifdef __cplusplus
}
#endif

#endif
