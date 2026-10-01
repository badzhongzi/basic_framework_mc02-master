#ifndef DMMOTOR_H
#define DMMOTOR_H
#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define DM_MOTOR_CNT 8

/*
 * MIT协议的映射范围(对称,±MAX),必须和达妙上位机里给电机设置的PMAX/VMAX/TMAX一致,
 * 否则收发的位置/速度/力矩会按比例错位.以下为占位值,实物到了用上位机读出来核对.
 * 单位: 位置rad,速度rad/s,力矩N·m,均为减速器输出端
 */
#define DM6248P_P_MAX 12.566f
#define DM6248P_V_MAX 20.0f
#define DM6248P_T_MAX 120.0f

#define DM4340P_P_MAX 12.5f
#define DM4340P_V_MAX 10.0f
#define DM4340P_T_MAX 28.0f

#define DM4310_P_MAX 12.5f
#define DM4310_V_MAX 30.0f
#define DM4310_T_MAX 10.0f

// 未指定型号时使用的映射范围(沿用原来的默认值)
#define DM_P_MIN  (-12.5f)
#define DM_P_MAX  12.5f
#define DM_V_MIN  (-45.0f)
#define DM_V_MAX  45.0f
#define DM_T_MIN  (-18.0f)
#define DM_T_MAX   18.0f

// 模式指令(使能/清错)的最小发送间隔,单位为电机任务周期(2ms),即100ms一次
#define DM_MODE_CMD_INTERVAL 50

/* 反馈帧D[0]高4位的电机状态 */
typedef enum
{
    DM_STATE_DISABLED = 0x0,     // 失能
    DM_STATE_ENABLED = 0x1,      // 使能
    DM_ERR_OVER_VOLTAGE = 0x8,   // 超压
    DM_ERR_UNDER_VOLTAGE = 0x9,  // 欠压
    DM_ERR_OVER_CURRENT = 0xA,   // 过电流
    DM_ERR_MOS_OVER_TEMP = 0xB,  // MOS过温
    DM_ERR_COIL_OVER_TEMP = 0xC, // 电机线圈过温
    DM_ERR_COMM_LOST = 0xD,      // 通讯丢失
    DM_ERR_OVERLOAD = 0xE,       // 过载
} DMMotor_State_e;

typedef struct
{
    float p_max;
    float v_max;
    float t_max;
} DM_Motor_Limit_s;

typedef struct
{
    uint8_t id;
    uint8_t state; // 取值见DMMotor_State_e
    float velocity;
    float last_position;
    float position;
    float torque;
    float T_Mos;
    float T_Rotor;
    int32_t total_round;
    uint32_t feedback_cnt; // 收到的反馈帧数,为0说明从未收到过反馈
}DM_Motor_Measure_s;

typedef struct
{
    uint16_t position_des;
    uint16_t velocity_des;
    uint16_t torque_des;
    uint16_t Kp;
    uint16_t Kd;
}DMMotor_Send_s;
typedef struct
{
    DM_Motor_Measure_s measure;
    Motor_Control_Setting_s motor_settings;
    Motor_Type_e motor_type;
    DM_Motor_Limit_s limit; // MIT映射范围,由motor_type决定
    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;
    float *other_angle_feedback_ptr;
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;
    float *current_feedforward_ptr;
    float pid_ref;
    Motor_Working_Type_e stop_flag;
    uint8_t clear_error_flag; // 上层请求清除错误,由电机任务发送后清零
    uint16_t mode_cmd_cnt;    // 距上次发送模式指令(使能/清错)的任务周期数
    CANInstance *motor_can_instace;
    DaemonInstance* motor_daemon;
    uint32_t lost_cnt;
}DMMotorInstance;

typedef enum
{
    DM_CMD_MOTOR_MODE = 0xfc,   // 使能,会响应指令
    DM_CMD_RESET_MODE = 0xfd,   // 停止
    DM_CMD_ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
    DM_CMD_CLEAR_ERROR = 0xfb // 清除电机过热错误
}DMMotor_Mode_e;

/**
 * @brief 注册达妙电机(MIT模式)
 *        can_init_config.tx_id 填电机的CAN_ID, rx_id 填电机的MST_ID(反馈帧的ID)
 *        motor_type 填 DM6248P/DM4340P/DM4310, 决定MIT映射范围
 *        不会发送零位设置指令, 零位由达妙上位机预先设置好
 */
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config);

void DMMotorSetRef(DMMotorInstance *motor, float ref);

void DMMotorOuterLoop(DMMotorInstance *motor,Closeloop_Type_e closeloop_type);

void DMMotorEnable(DMMotorInstance *motor);

void DMMotorStop(DMMotorInstance *motor);
/**
 * @brief 把当前位置设为编码器零位. 会阻塞100ms, 只用于调试,
 *        零位由达妙上位机预先设置, 不要放进上电流程
 */
void DMMotorCaliEncoder(DMMotorInstance *motor);
/**
 * @brief 请求清除电机错误(过压/欠压/过流/过温等),由电机任务按DM_MODE_CMD_INTERVAL限频发送
 *        驱动不会自动清错,必须由上层确认后调用
 */
void DMMotorClearError(DMMotorInstance *motor);
void DMMotorControlInit();
#endif // !DMMOTOR
