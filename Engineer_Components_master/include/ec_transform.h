#ifndef ENGINEER_COMPONENTS_C_TRANSFORM_H
#define ENGINEER_COMPONENTS_C_TRANSFORM_H

#include "ec_math.h"

/*
 * 姿态表示转换与齐次变换操作。
 *
 * 支持的姿态表示：
 *   - 旋转矩阵 (Mat3)       3×3 正交矩阵，det = +1
 *   - RPY 欧拉角 (Vec3)      [yaw, pitch, roll]，Z-Y-X 旋转顺序
 *   - 轴角 (float[4])        [axis_x, axis_y, axis_z, angle]
 *   - 四元数 (Quat)          [w, x, y, z]
 *   - 齐次变换 (Mat4)        4×4 刚体变换矩阵
 *   - 位姿旋量 (float[6])    [x, y, z, rx, ry, rz]，即平移向量与轴角旋转向量
 *
 * 所有角度参数和返回值单位均为弧度（rad）。
 * 长度单位由应用统一，本模块不做换算。
 * MDH 变换采用 MDH（Modified DH）约定，与原 C++ 工程一致。
 */

#ifdef __cplusplus
extern "C" {
#endif

/* ── 旋转矩阵 ↔ RPY 欧拉角 ── */
/* RPY 顺序为 [yaw, pitch, roll]，对应绕固定轴 Z-Y-X 旋转（等价于体轴 X-Y-Z）。 */
/* pitch = ±π/2 时存在万向锁，yaw 和 roll 退化为一个自由度。 */

/* 旋转矩阵 → RPY：r → [yaw, pitch, roll]。 */
EC_Vec3 ec_r_to_rpy(EC_Mat3 r);

/* RPY → 旋转矩阵：[yaw, pitch, roll] → r。 */
EC_Mat3 ec_rpy_to_r(EC_Vec3 rpy);

/* ── 旋转矩阵 ↔ 轴角 ── */
/* 轴角表示：绕单位轴 (axis_x, axis_y, axis_z) 旋转 angle 弧度。 */
/* 前三项为无量纲单位轴分量，第四项为旋转角度（rad）。 */

/* 旋转矩阵 → 单位轴（仅方向，不含角度）。 */
EC_Vec3 ec_r_to_angvec(EC_Mat3 r);

/* 旋转矩阵 → 轴角 [axis_x, axis_y, axis_z, angle]；angle 单位 rad。 */
/* 零旋转时输出 [1, 0, 0, 0]；π 附近使用旋转矩阵对角线稳定恢复旋转轴。 */
void ec_r_to_angvec4(EC_Mat3 r, float out[4]);

/* 轴角 → 旋转矩阵：angvec = [axis_x, axis_y, axis_z, angle]。 */
EC_Mat3 ec_angvec_to_r(const float angvec[4]);

/* ── 四元数 ↔ RPY / 轴角 ── */

/* 四元数 → RPY：q → [yaw, pitch, roll]。 */
EC_Vec3 ec_quat_to_rpy(EC_Quat q);

/* RPY → 四元数：[yaw, pitch, roll] → q。 */
EC_Quat ec_rpy_to_quat(EC_Vec3 rpy);

/* 四元数 → 轴角：q → [axis_x, axis_y, axis_z, angle]。 */
void ec_quat_to_angvec(EC_Quat q, float out[4]);

/* 轴角 → 四元数：[axis_x, axis_y, axis_z, angle] → q。 */
EC_Quat ec_angvec_to_quat(const float angvec[4]);

/* ── 齐次变换矩阵的提取与构造 ── */
/* 齐次变换 T = | R  p | ，R 为 3×3 旋转矩阵，p 为 3×1 平移向量。 */
/*              | 0  1 | */

/* 从齐次变换提取 3×3 旋转矩阵。 */
EC_Mat3 ec_t_to_r(EC_Mat4 t);

/* 从齐次变换提取平移向量；长度单位继承输入齐次矩阵。 */
EC_Vec3 ec_t_to_p(EC_Mat4 t);

/* 纯旋转齐次变换：平移为零，旋转为 r。 */
/* T = | r  0 | */
/*     | 0  1 | */
EC_Mat4 ec_r_to_t(EC_Mat3 r);

/* 纯平移齐次变换：旋转为单位阵，平移为 p。 */
/* T = | I  p | */
/*     | 0  1 | */
EC_Mat4 ec_p_to_t(EC_Vec3 p);

/* 由旋转矩阵 r 和平移向量 p 构造齐次变换。 */
/* T = | r  p | */
/*     | 0  1 | */
EC_Mat4 ec_rp_to_t(EC_Mat3 r, EC_Vec3 p);

/* 刚体齐次变换求逆：利用旋转矩阵正交性高效计算。 */
/* T^{-1} = | R^T  -R^T×p | */
/*          | 0      1    | */
EC_Mat4 ec_t_inverse(EC_Mat4 t);

/* ── 齐次变换 ↔ 其他姿态表示 ── */

/* 齐次变换 → RPY：提取旋转矩阵后转为 [yaw, pitch, roll]。 */
EC_Vec3 ec_t_to_rpy(EC_Mat4 t);

/* RPY → 齐次变换：构造纯旋转齐次变换。 */
EC_Mat4 ec_rpy_to_t(EC_Vec3 rpy);

/* 齐次变换 → 四元数：提取旋转矩阵后转为四元数。 */
EC_Quat ec_t_to_quat(EC_Mat4 t);

/* 四元数 → 齐次变换：构造纯旋转齐次变换。 */
EC_Mat4 ec_quat_to_t(EC_Quat q);

/* ── 齐次变换 ↔ 位姿旋量 ── */
/* 位姿旋量 twist = [x, y, z, rx, ry, rz]： */
/*   前三项为平移分量（长度单位由应用统一）； */
/*   后三项为旋转分量 = angle × axis（单位 rad，方向为旋转轴）。 */

/* 齐次变换 → 位姿旋量 [x, y, z, rx, ry, rz]。 */
void ec_t_to_twist(EC_Mat4 t, float out[6]);

/* 运动旋量 → 齐次变换；输入单位同 ec_t_to_twist。 */
EC_Mat4 ec_twist_to_t(const float twist[6]);

/* ── MDH 变换 ── */
/* 按 MDH（Modified DH，Craig）约定生成单节连杆的齐次变换矩阵： */
/*   T = Rot(x, alpha) × Trans(x, a) × Rot(z, theta) × Trans(z, d) */
/* alpha、theta 单位为 rad；a、d 单位为长度（由应用统一，示例使用 m）。 */
/* 注意：Standard DH 的顺序为 Rot(z,θ)·Trans(z,d)·Trans(x,a)·Rot(x,α)，与本约定不同。 */
EC_Mat4 ec_mdh_to_transform(float theta, float d, float a, float alpha);

#ifdef __cplusplus
}
#endif

#endif
