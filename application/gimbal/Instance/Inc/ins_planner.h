#ifndef EC_DEMO_6DOF_INS_PLANNER_H
#define EC_DEMO_6DOF_INS_PLANNER_H

#include "ins_arm.h"

/*
 * 控制周期，单位 s。
 * 唯一定义处在 ins_arm.h 的 INS_ARM_CONTROL_PERIOD, 这里只做别名, 避免
 * 两处各写一个字面量再各自漂移 (改动前这里是 0.002f, ins_arm.c 里却是
 * 0.005f, 而真实调用率是 2 ms).
 */
#define INS_PLANNER_CONTROL_PERIOD INS_ARM_CONTROL_PERIOD
/* 多节点关节空间命令最大节点数。 */
#define INS_PLANNER_MAX_NODES 8u

/* 构造一个单节点关节空间命令；target 单位 rad，duration 单位 s。 */
bool ins_planner_make_joint_command(EC_ArmFsmCmd *command,
                                    const float target[INS_ARM_6DOF_COUNT],
                                    float duration);

/*
 * 构造多节点关节空间命令。
 * nodes[i] 是第 i 个目标节点，关节角单位 rad；traj_time[i] 是从前一
 * 个节点到 nodes[i] 的时间，单位 s。整条路径的隐式起点是当前反馈角。
 */
bool ins_planner_make_multinode_joint_command(
    EC_ArmFsmCmd *command,
    const float nodes[][INS_ARM_6DOF_COUNT], const float traj_time[],
    uint8_t node_count);

/* 通用无规划接口，构造无规划命令，node 可为关节空间或笛卡尔空间节点。 */
bool ins_planner_make_none_command(EC_ArmFsmCmd *command,
                                   const EC_ArmCmd *node);

/* 无规划关节空间便捷接口，构造关节空间无规划命令，target 单位 rad。 */
bool ins_planner_make_none_joint_command(
    EC_ArmFsmCmd *command,
    const float target[INS_ARM_6DOF_COUNT]);

/* 无规划笛卡尔空间便捷接口，构造笛卡尔空间无规划命令，位置长度单位 m，四元数顺序 [w,x,y,z]。 */
bool ins_planner_make_none_cartesian_command(EC_ArmFsmCmd *command,
                                             EC_CartesianCmd target);

/* 构造一个两节点笛卡尔直线命令；位置长度单位 m，姿态四元数为 [w,x,y,z]。 */
bool ins_planner_make_linear_command(EC_ArmFsmCmd *command,
                                     EC_CartesianCmd start,
                                     EC_CartesianCmd end, float duration);
#endif /* EC_DEMO_6DOF_INS_PLANNER_H */
