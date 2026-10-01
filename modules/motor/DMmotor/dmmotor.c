#include "dmmotor.h"
#include "memory.h"
#include "general_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_log.h"
#include "bsp_dwt.h"

static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];
static osThreadId dm_task_handle[DM_MOTOR_CNT];
static char dm_task_name[DM_MOTOR_CNT][4]; // "dm0".."dm7", 任务创建时名字会被拷进TCB,这里只是临时存放

/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/* 根据电机型号确定MIT映射范围,未知型号沿用原来的默认值 */
static void DMMotorSetLimit(DMMotorInstance *motor)
{
    switch (motor->motor_type)
    {
    case DM6248P:
        motor->limit = (DM_Motor_Limit_s){DM6248P_P_MAX, DM6248P_V_MAX, DM6248P_T_MAX};
        break;
    case DM4340P:
        motor->limit = (DM_Motor_Limit_s){DM4340P_P_MAX, DM4340P_V_MAX, DM4340P_T_MAX};
        break;
    case DM4310:
        motor->limit = (DM_Motor_Limit_s){DM4310_P_MAX, DM4310_V_MAX, DM4310_T_MAX};
        break;
    default:
        motor->limit = (DM_Motor_Limit_s){DM_P_MAX, DM_V_MAX, DM_T_MAX};
        break;
    }
}

static void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor)
{
    memset(motor->motor_can_instace->tx_buff, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    CANTransmit(motor->motor_can_instace, 1);
}

static void DMMotorDecode(CANInstance *motor_can)
{
    uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id; // 将can实例中保存的id转换成电机实例的指针
    DM_Motor_Measure_s *measure = &(motor->measure);
    DM_Motor_Limit_s *limit = &motor->limit;
    float dir = motor->motor_settings.feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE ? -1.0f : 1.0f;

    DaemonReload(motor->motor_daemon);

    measure->id = rxbuff[0] & 0x0f;
    measure->state = rxbuff[0] >> 4;

    measure->last_position = measure->position;
    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->position = dir * uint_to_float(tmp, -limit->p_max, limit->p_max, 16);

    tmp = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
    measure->velocity = dir * uint_to_float(tmp, -limit->v_max, limit->v_max, 12);

    tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    measure->torque = dir * uint_to_float(tmp, -limit->t_max, limit->t_max, 12);

    measure->T_Mos = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];
    measure->feedback_cnt++;
}

static void DMMotorLostCallback(void *motor_ptr)
{
    DMMotorInstance *motor = (DMMotorInstance *)motor_ptr;
    uint16_t can_bus = motor->motor_can_instace->can_handle == &hcan1 ? 1 : (motor->motor_can_instace->can_handle == &hcan2 ? 2 : 3);
    (void)can_bus; // 关闭日志系统时LOGWARNING为空,避免unused警告
    // daemon离线后每10ms调用一次,日志限频到1s一条
    if (motor->lost_cnt++ % 100 == 0)
        LOGWARNING("[dm_motor] Motor lost, can bus [%d] , id [%d]", can_bus, motor->motor_can_instace->tx_id);
}

void DMMotorCaliEncoder(DMMotorInstance *motor)
{
    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.1);
}

DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    if (idx >= DM_MOTOR_CNT) // 超过最大实例数
    {
        while (1)
            LOGERROR("[dm_motor] DM motor instance exceeded MAX num [%d]", DM_MOTOR_CNT);
    }
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    memset(motor, 0, sizeof(DMMotorInstance));

    motor->motor_settings = config->controller_setting_init_config;
    motor->motor_type = config->motor_type;
    DMMotorSetLimit(motor);
    PIDInit(&motor->current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->angle_PID, &config->controller_param_init_config.angle_PID);
    motor->other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;

    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id = motor;
    motor->motor_can_instace = CANRegister(&config->can_init_config);

    Daemon_Init_Config_s conf = {
        .callback = DMMotorLostCallback,
        .owner_id = motor,
        .reload_count = 10, // 100ms
    };
    motor->motor_daemon = DaemonRegister(&conf);

    // 不在这里发使能: 初始化时电机未必已经上电, 使能指令由电机任务在收到"失能"反馈或没有反馈时周期性补发
    // 也不再发零位设置指令, 零位由达妙上位机预先设置
    DMMotorEnable(motor);
    motor->mode_cmd_cnt = DM_MODE_CMD_INTERVAL; // 任务第一个周期就发一次使能
    dm_motor_instance[idx++] = motor;
    return motor;
}

void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    motor->pid_ref = ref;
}

void DMMotorEnable(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void DMMotorStop(DMMotorInstance *motor)//不使用失能模式是因为需要收到反馈,停止时发0力矩且Kp=Kd=0,电机不出力
{
    motor->stop_flag = MOTOR_STOP;
}

void DMMotorClearError(DMMotorInstance *motor)
{
    motor->clear_error_flag = 1;
}

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}

/**
 * @brief 需要发模式指令时发送并返回1, 本周期不再发MIT控制帧(两者共用tx_buff, 且都会触发一帧反馈)
 *        - 上层请求清错: 发清错指令, 电机清错后回到失能状态, 之后由下面的分支重新使能
 *        - 失能或还没有收到过反馈: 补发使能指令, 电机晚于主控上电/掉电重启时都能自动重新使能
 *        - 处于错误状态: 不自动清错也不使能, 必须由上层确认后调用DMMotorClearError()
 *        以上都按DM_MODE_CMD_INTERVAL限频, 其余周期照常发MIT帧以获取反馈
 */
static uint8_t DMMotorModeManage(DMMotorInstance *motor)
{
    DM_Motor_Measure_s *measure = &motor->measure;

    if (motor->mode_cmd_cnt < DM_MODE_CMD_INTERVAL)
    {
        motor->mode_cmd_cnt++;
        return 0;
    }
    if (motor->clear_error_flag)
    {
        motor->clear_error_flag = 0;
        motor->mode_cmd_cnt = 0;
        DMMotorSetMode(DM_CMD_CLEAR_ERROR, motor);
        return 1;
    }
    if (measure->feedback_cnt == 0 || !DaemonIsOnline(motor->motor_daemon) || measure->state == DM_STATE_DISABLED)
    {
        motor->mode_cmd_cnt = 0;
        DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
        return 1;
    }
    return 0;
}

//@Todo: 目前只实现了力控，更多位控PID等请自行添加
void DMMotorTask(void const *argument)
{
    float set;
    DMMotorInstance *motor = (DMMotorInstance *)argument;
    DM_Motor_Limit_s *limit = &motor->limit;
    Motor_Control_Setting_s *setting = &motor->motor_settings;
    DMMotor_Send_s motor_send_mailbox;
    uint32_t wake_time = osKernelSysTick();
    while (1)
    {
        if (DMMotorModeManage(motor))
        {
            osDelayUntil(&wake_time, 2);
            continue;
        }

        set = motor->pid_ref;
        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            set *= -1;
        LIMIT_MIN_MAX(set, -limit->t_max, limit->t_max);
        if (motor->stop_flag == MOTOR_STOP)
            set = 0;

        motor_send_mailbox.position_des = float_to_uint(0, -limit->p_max, limit->p_max, 16);
        motor_send_mailbox.velocity_des = float_to_uint(0, -limit->v_max, limit->v_max, 12);
        motor_send_mailbox.torque_des = float_to_uint(set, -limit->t_max, limit->t_max, 12);
        motor_send_mailbox.Kp = 0;
        motor_send_mailbox.Kd = 0;

        motor->motor_can_instace->tx_buff[0] = (uint8_t)(motor_send_mailbox.position_des >> 8);
        motor->motor_can_instace->tx_buff[1] = (uint8_t)(motor_send_mailbox.position_des);
        motor->motor_can_instace->tx_buff[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 4);
        motor->motor_can_instace->tx_buff[3] = (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) | (motor_send_mailbox.Kp >> 8));
        motor->motor_can_instace->tx_buff[4] = (uint8_t)(motor_send_mailbox.Kp);
        motor->motor_can_instace->tx_buff[5] = (uint8_t)(motor_send_mailbox.Kd >> 4);
        motor->motor_can_instace->tx_buff[6] = (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) | (motor_send_mailbox.torque_des >> 8));
        motor->motor_can_instace->tx_buff[7] = (uint8_t)(motor_send_mailbox.torque_des);

        CANTransmit(motor->motor_can_instace, 1);

        osDelayUntil(&wake_time, 2);
    }
}
void DMMotorControlInit()
{
    // 遍历所有电机实例,创建任务
    if (!idx)
        return;
    for (size_t i = 0; i < idx; i++)
    {
        dm_task_name[i][0] = 'd';
        dm_task_name[i][1] = 'm';
        dm_task_name[i][2] = (char)('0' + i);
        dm_task_name[i][3] = '\0';
        osThreadDef_t dm_task_def = {
            .name = dm_task_name[i],
            .pthread = DMMotorTask,
            .tpriority = osPriorityNormal,
            .instances = 0,
            .stacksize = 128,
        };
        dm_task_handle[i] = osThreadCreate(&dm_task_def, dm_motor_instance[i]);
    }
}
