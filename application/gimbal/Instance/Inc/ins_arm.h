#ifndef EC_DEMO_6DOF_INS_ARM_H
#define EC_DEMO_6DOF_INS_ARM_H

#include "engineer_components_c.h"

#define INS_ARM_6DOF_COUNT 6u
#if EC_MAX_JOINTS < INS_ARM_6DOF_COUNT
#error "EC_MAX_JOINTS must be at least 6 for the 6DOF demo"
#endif

/*
 * 机械臂控制周期, 唯一定义处.
 *
 * 必须与 robot_task.h 中 StartROBOTTASK 的 osDelayUntil(&robot_wake_time, N)
 * 保持一致: RobotTask 以严格 N ms 周期唤醒, GimbalTask 在其中同步调用,
 * 因此机械臂的控制周期就是这个 N. FreeRTOSConfig.h 的 configTICK_RATE_HZ
 * 为 1000, 所以 1 tick = 1 ms, 二者可以直接换算.
 *
 * 改这里之前先改 robot_task.h, 否则规划器的时间标尺会和真实调用率脱节:
 * 周期偏大会让整条轨迹按比例变慢, 同时 q_dot 偏小一个比例、q_ddot 偏小
 * 该比例的平方, 而 q_ddot 直接进逆动力学前馈, 惯性项会被系统性低估.
 */
#define INS_ARM_CONTROL_TICK_MS 2u
#define INS_ARM_CONTROL_PERIOD (0.001f * (float)INS_ARM_CONTROL_TICK_MS)

/*
 * 6DOF 机械臂实例。
 */
typedef struct {
  EC_SerialLink serial;   /* MDH 连杆模型和正运动学缓存。 */
  EC_Arm arm;             /* 反馈、命令、规划器和参考角状态。 */
} InsArm;

/* 使用原始 6DOF demo 的 MDH、关节限位和 ikine函数 参数初始化实例。
  * 图方便提前写死了, 未来可能会直接urdf到py脚本再到函数内部一条龙实现, 避免手动抄写修改
  * 如果有多臂需求, 只需自行增加一个ins_arm_init_second()即可, 其他接口都是能通用的 */

bool ins_arm_init(InsArm *instance);

/* 写入关节反馈；单位 rad。写入后需调用 ins_arm_update() 刷新位姿。 */
void ins_arm_set_feedback(InsArm *instance, const float joint[INS_ARM_6DOF_COUNT]);

/* 根据反馈角计算正运动学并刷新末端位姿。 */
void ins_arm_update(InsArm *instance);

/* 读取当前 EC_Arm 状态；返回的指针由 instance 持有。 */
EC_Arm *ins_arm_state(InsArm *instance);
/* 读取当前 EC_Arm 状态；返回的指针由 instance 持有。 */
const EC_Arm *ins_arm_const_state(const InsArm *instance);

/* 用指定关节角计算末端位姿；不会改变 EC_Arm 的反馈状态。 */
bool ins_arm_fkine(const InsArm *instance,
                   const float joint[INS_ARM_6DOF_COUNT],
                   EC_CartesianCmd *cartesian);

/* 判断当前命令是否已经完成。 */
bool ins_arm_motion_finished(const InsArm *instance);

#endif /* EC_DEMO_6DOF_INS_ARM_H */
