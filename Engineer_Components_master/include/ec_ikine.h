#ifndef ENGINEER_COMPONENTS_C_IKINE_H
#define ENGINEER_COMPONENTS_C_IKINE_H

#include "ec_types.h"

/*
 * 特定机械臂几何结构的解析逆运动学求解器。
 *
 * 提供 5 种与特定 MDH 构型绑定的解析逆解函数：
 *   - ec_ikine1: 6-DOF，4 个连杆长度参数，最多 8 组解
 *   - ec_ikine2: 6-DOF 变体，4 个连杆长度参数，最多 8 组解
 *   - ec_ikine3: 7-DOF，4 个连杆长度 + 指定 theta6/theta7，最多 4 组解
 *   - ec_ikine4: 7-DOF 变体，6 个连杆长度 + 指定 theta7，最多 8 组解
 *   - ec_ikine5: 6-DOF，3 个连杆长度参数，最多 8 组解
 *
 * 这些解析式与特定 MDH 结构强绑定，使用前必须确认机器人几何参数匹配。
 * 对于其它构型的机械臂，应通过 EC_Arm.solve_ikine 回调注入自定义逆解。
 *
 * 返回值约定：
 *   - 返回值为有效候选解的数量
 *   - solution 每行是一组完整的关节角候选解，单位 rad
 *   - 部分不满足数值范围检查的候选解会被跳过；奇异位形的处理取决于具体求解器
 *
 * 角度单位为 rad，长度单位由应用统一（示例使用 m）。
 */

#ifdef __cplusplus
extern "C" {
#endif

/* ── 6-DOF 逆解 ── */

/* 6-DOF 逆解（构型 1）：标准 6 轴机械臂。
 * cmd:     目标笛卡尔位姿（位置 + 四元数姿态）。
 * length:  连杆几何长度参数数组 [l0, l1, l2, l3]，共 4 个，长度单位由应用统一。
 *          具体含义与原 C++ ikine1 对应的 MDH 构型绑定。
 * solution: 输出候选解数组，8 行 × 6 列，每行一组关节角 [θ1~θ6]，单位 rad。
 * 返回值:  有效解数量（0~8）。 */
uint8_t ec_ikine1(EC_ModuleCartesianCmd cmd, const float length[4],
                  float solution[8][6]);

/* 6-DOF 逆解（构型 2）：带偏置的 6 轴机械臂变体。
 * 参数含义与 ec_ikine1 相同，但内部求解流程对应不同的 MDH 构型。
 * length:  [l0, l1, l2, l3]，共 4 个长度参数。
 * solution: 8 行 × 6 列。
 * 返回值:  有效解数量（0~8）。 */
uint8_t ec_ikine2(EC_ModuleCartesianCmd cmd, const float length[4],
                  float solution[8][6]);

/* 6-DOF 逆解（构型 5）：3 参数简化模型。
 * length:  连杆几何长度参数数组 [l0, l1, l2]，共 3 个。
 *          对应原 C++ ikine5 的 MDH 构型。
 * solution: 8 行 × 6 列。
 * 返回值:  有效解数量（0~8）。 */
uint8_t ec_ikine5(EC_ModuleCartesianCmd cmd, const float length[3],
                  float solution[8][6]);

/* ── 7-DOF 逆解 ── */

/* 7-DOF 逆解（构型 3）：冗余机械臂，需指定末两关节角。
 * 7 自由度机械臂的解空间是连续的，通过固定 theta6 和 theta7
 * 将其参数化为有限组离散解。
 * cmd:     目标笛卡尔位姿。
 * length:  连杆几何长度参数数组 [l0, l1, l2, l3]，共 4 个。
 * theta6:  第 6 关节指定角度（rad），作为冗余参数。
 * theta7:  第 7 关节指定角度（rad），作为冗余参数。
 * solution: 输出候选解数组，4 行 × 7 列，每行 [θ1~θ7]，单位 rad。
 * 返回值:  有效解数量（0~4）。 */
uint8_t ec_ikine3(EC_ModuleCartesianCmd cmd, const float length[4],
                  float theta6, float theta7, float solution[4][7]);

/* 7-DOF 逆解（构型 4）：冗余机械臂变体，需指定末关节角。
 * 通过固定 theta7 将解空间参数化。
 * cmd:     目标笛卡尔位姿。
 * length:  连杆几何长度参数数组 [l0, l1, l2, l3, l4, l5]，共 6 个。
 * theta7:  第 7 关节指定角度（rad），作为冗余参数。
 * solution: 输出候选解数组，8 行 × 7 列，每行 [θ1~θ7]，单位 rad。
 * 返回值:  有效解数量（0~8）。 */
uint8_t ec_ikine4(EC_ModuleCartesianCmd cmd, const float length[6],
                  float theta7, float solution[8][7]);

#ifdef __cplusplus
}
#endif

#endif
