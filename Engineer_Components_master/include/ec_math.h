#ifndef ENGINEER_COMPONENTS_C_MATH_H
#define ENGINEER_COMPONENTS_C_MATH_H

#include "ec_types.h"

/*
 * 基础数学工具：限幅/归一化、三维向量、3x3/4x4 矩阵和四元数运算。
 *
 * 所有角度参数和返回值单位均为弧度（rad）。
 * 向量和矩阵均采用行优先存储，与 ec_types.h 中的定义一致。
 * 四元数顺序为 [w, x, y, z]（标量在前）。
 */

#ifdef __cplusplus
extern "C" {
#endif

/* ── 标量工具 ── */

/* 将 value 限幅到 [min_value, max_value] 区间。 */
float ec_limit(float value, float min_value, float max_value);

/* 将 value 归一化到周期区间 [lower, upper) 内。 */
/* 典型用途：将角度映射到 [-π, π) 或 [0, 2π) 等周期范围。 */
float ec_norm_period(float lower, float upper, float value);

/* 将 angle 归一化到 [-π, π) 区间。 */
float ec_norm_angle(float angle);

/* 判断两个浮点数在给定容差 epsilon 内是否近似相等。 */
bool ec_float_equal(float a, float b, float epsilon);

/* 将 *value 夹紧到 [-1, 1]；若原始值偏离超过 epsilon 则返回 false。 */
/* 典型用途：修正 acosf/asinf 参数因浮点误差超出 [-1, 1] 的情况。 */
bool ec_sincos_bound(float *value, float epsilon);

/* ── 三维向量运算 ── */

/* 构造三维向量 (x, y, z)。 */
EC_Vec3 ec_vec3(float x, float y, float z);

/* 向量加法：a + b。 */
EC_Vec3 ec_vec3_add(EC_Vec3 a, EC_Vec3 b);

/* 向量减法：a - b。 */
EC_Vec3 ec_vec3_sub(EC_Vec3 a, EC_Vec3 b);

/* 向量数乘：a × scalar。 */
EC_Vec3 ec_vec3_scale(EC_Vec3 a, float scalar);

/* 向量点乘（内积）：a · b。 */
float ec_vec3_dot(EC_Vec3 a, EC_Vec3 b);

/* 向量叉乘（外积）：a × b。 */
EC_Vec3 ec_vec3_cross(EC_Vec3 a, EC_Vec3 b);

/* 向量 L2 范数（欧几里得长度）：||a||。 */
float ec_vec3_norm(EC_Vec3 a);

/* 向量归一化：a / ||a||；零向量返回零向量。 */
EC_Vec3 ec_vec3_normalize(EC_Vec3 a);

/* ── 3×3 矩阵运算 ── */

/* 将矩阵所有元素清零。 */
void ec_mat3_zero(EC_Mat3 *out);

/* 将矩阵设为单位阵。 */
void ec_mat3_identity(EC_Mat3 *out);

/* 矩阵乘法：a × b。 */
EC_Mat3 ec_mat3_mul(EC_Mat3 a, EC_Mat3 b);

/* 矩阵-向量乘法：a × b，结果为三维向量。 */
EC_Vec3 ec_mat3_vec(EC_Mat3 a, EC_Vec3 b);

/* 矩阵转置。 */
EC_Mat3 ec_mat3_transpose(EC_Mat3 a);

/* 反对称矩阵（hat map）：由向量 v 构造 3×3 反对称矩阵 [v]×。 */
/* 满足 [v]× × u = v × u（叉乘），用于雅可比和旋转表示。 */
EC_Mat3 ec_hat(EC_Vec3 v);

/* ── 4×4 齐次变换矩阵运算 ── */

/* 将矩阵所有元素清零。 */
void ec_mat4_zero(EC_Mat4 *out);

/* 将矩阵设为单位阵。 */
void ec_mat4_identity(EC_Mat4 *out);

/* 矩阵乘法：a × b，用于组合齐次变换。 */
EC_Mat4 ec_mat4_mul(EC_Mat4 a, EC_Mat4 b);

/* 由旋转矩阵 r 和平移向量 p 构造齐次变换矩阵：
 * | r  p |
 * | 0  1 | */
EC_Mat4 ec_mat4_from_rp(EC_Mat3 r, EC_Vec3 p);

/* 刚体齐次变换矩阵求逆：
 * | R  p | -1   | R^T  -R^T×p |
 * | 0  1 |    = | 0      1    | */
EC_Mat4 ec_mat4_inverse(EC_Mat4 t);

/* ── 四元数运算 ── */
/* 四元数顺序为 [w, x, y, z]，w 为标量部分，(x, y, z) 为向量部分。 */

/* 构造四元数 (w, x, y, z)。 */
EC_Quat ec_quat(float w, float x, float y, float z);

/* 单位四元数 (1, 0, 0, 0)，表示零旋转。 */
EC_Quat ec_quat_identity(void);

/* 四元数归一化：q / ||q||。 */
EC_Quat ec_quat_normalize(EC_Quat q);

/* 四元数 Hamilton 乘法：a ⊗ b，组合两次旋转。 */
EC_Quat ec_quat_mul(EC_Quat a, EC_Quat b);

/* 四元数共轭：q* = (w, -x, -y, -z)。 */
/* 单位四元数的共轭等于其逆。 */
EC_Quat ec_quat_conjugate(EC_Quat q);

/* 四元数逆：q^{-1} = q* / ||q||²。 */
EC_Quat ec_quat_inverse(EC_Quat q);

/* 四元数 L2 范数：||q||。 */
float ec_quat_norm(EC_Quat q);

/* 四元数转 3×3 旋转矩阵。 */
EC_Mat3 ec_quat_to_mat3(EC_Quat q);

/* 3×3 旋转矩阵转四元数。 */
EC_Quat ec_mat3_to_quat(EC_Mat3 r);

#ifdef __cplusplus
}
#endif

#endif