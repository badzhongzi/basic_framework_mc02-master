#ifndef GIMBAL_H
#define GIMBAL_H

/*
 * gimbal应用: 6自由度机械臂的关节层.
 *
 * 只做三件事: 注册关节电机, 把cmd发来的 Gimbal_Ctrl_Cmd_s (模式 + 目标角)
 * 交给 ArmMpcTask, 把机械臂状态打包成 Gimbal_Upload_Data_s 回传.
 * 轨迹、前馈、MPC和降级逻辑全部在 Instance/arm_mpc_task 里.
 * 用哪个遥控器、预设点位、优先级仲裁都是cmd层的事, 这里不处理.
 */

/**
 * @brief 初始化机械臂,会被RobotInit()调用
 *
 */
void GimbalInit();

/**
 * @brief 机械臂任务, 周期必须是 INS_ARM_CONTROL_TICK_MS
 *
 */
void GimbalTask();

#endif // GIMBAL_H
