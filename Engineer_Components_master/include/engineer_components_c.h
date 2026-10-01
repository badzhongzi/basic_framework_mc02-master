#ifndef ENGINEER_COMPONENTS_C_H
#define ENGINEER_COMPONENTS_C_H

/*
 * Engineer-Components C 版本总入口头文件。
 *
 * 一行包含即可使用框架全部功能：
 *   #include "engineer_components_c.h"
 *
 * 包含顺序（自底向上，后者依赖前者）：
 *   ec_types.h       基础类型定义（向量、矩阵、四元数、MDH 参数、规划器结构体等）
 *   ec_matrix.h      通用定长矩阵（最大 16×16，行优先）
 *   ec_math.h        Vec3/Mat3/Mat4/Quat 运算 + 角度归一化/限幅
 *   ec_operation.h   对 math 的语义化封装（cross、dot、HTM 乘法）
 *   ec_transform.h   姿态转换（RPY↔旋转矩阵↔四元数↔轴角↔齐次变换↔MDH 变换）
 *   ec_link.h        MDH 连杆、串联正运动学、雅可比、重力补偿和完整逆动力学
 *   ec_planner.h     线性/关节/笛卡尔轨迹规划器（逐周期在线插值）
 *   ec_traj.h        可求值轨迹计划（五次系数 + 绝对时间基准，支持任意时刻求值）
 *   ec_ikine.h       5 种特定构型的解析逆解
 *   ec_arm.h         机械臂状态机（整合以上所有模块）
 *
 * 编译前可覆盖的宏：
 *   EC_MAX_JOINTS           最大关节数（默认 16）
 *   EC_MAX_IK_SOLUTIONS     最大逆解候选解数（默认 8）
 *   EC_MATRIX_MAX_ROWS      通用矩阵最大行数（默认 16）
 *   EC_MATRIX_MAX_COLS      通用矩阵最大列数（默认 16）
 *   EC_GRAVITY_METHOD       重力补偿算法选择（默认 0）
 *                          0 = EC_GRAVITY_JACOBIAN_TRANSPOSE: 雅可比转置法, O(N²)
 *                          1 = EC_GRAVITY_NEWTON_EULER:       牛顿欧拉递推法, O(N)
 *                          两种方法计算同一个重力补偿项，浮点结果可能有微小舍入差异
 *                          编译时覆盖示例: -DEC_GRAVITY_METHOD=1
 *
 * 角度单位一律为 rad，四元数顺序 [w, x, y, z]，矩阵行优先。
 * 所有对象不使用堆分配，适合嵌入式环境。
 */

#include "ec_types.h"
#include "ec_matrix.h"
#include "ec_math.h"
#include "ec_operation.h"
#include "ec_transform.h"
#include "ec_link.h"
#include "ec_planner.h"
#include "ec_traj.h"
#include "ec_ikine.h"
#include "ec_arm.h"

#endif
