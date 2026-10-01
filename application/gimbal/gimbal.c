#include "gimbal.h"
#include "robot_def.h"
#include "message_center.h"
#include "dmmotor.h"
#include "arm_mpc_task.h"
#include "bsp_log.h"
#include <string.h>

_Static_assert(ARM_JOINT_NUM == ARM_MPC_JOINT_COUNT, "robot_def.h ARM_JOINT_NUM mismatch");
_Static_assert((int)ARM_MOVE_FOLLOW == (int)ARM_MPC_MOVE_FOLLOW && (int)ARM_MOVE_TIMED == (int)ARM_MPC_MOVE_TIMED,
               "robot_def.h arm_move_e mismatch");
_Static_assert(sizeof(Gimbal_Ctrl_Cmd_s) <= 255, "message_center data_len is uint8_t");
_Static_assert(sizeof(Gimbal_Upload_Data_s) <= 255, "message_center data_len is uint8_t");

/*
 * 关节电机CAN配置, 实物到了之后只改这张表.
 * tx_id 填电机的CAN_ID, rx_id 填电机的MST_ID(反馈帧ID), 在达妙上位机里查看/设置.
 *
 * 目前全部是占位值. tx_id 为 ARM_JOINT_ID_PLACEHOLDER 的关节不会注册电机,
 * 按"理想跟踪"参与计算(反馈取参考值): 同一条总线上 rx_id 重复注册会让
 * CANRegister() 死循环, 占位值全部相同, 所以不能直接注册.
 *
 * 负载: 每个电机 500Hz 收发各一帧, 6个电机挤在一条1Mbps总线上约占78%,
 * 实物接线时建议分到两条总线上.
 * MC02上底盘占用CAN1(hcan1), 发射占用CAN2(hcan2), 机械臂默认放在CAN3(hcan3).
 */
#define ARM_JOINT_ID_PLACEHOLDER 0x00

typedef struct
{
#ifdef FDCAN
    FDCAN_HandleTypeDef *can_handle;
#else
    CAN_HandleTypeDef *can_handle;
#endif
    uint32_t tx_id;
    uint32_t rx_id;
    Motor_Type_e motor_type;
} Arm_Joint_Motor_Config_s;

static const Arm_Joint_Motor_Config_s joint_motor_config[ARM_JOINT_NUM] = {
    {&hcan3, ARM_JOINT_ID_PLACEHOLDER, ARM_JOINT_ID_PLACEHOLDER, DM6248P}, // J0
    {&hcan3, ARM_JOINT_ID_PLACEHOLDER, ARM_JOINT_ID_PLACEHOLDER, DM4340P}, // J1
    {&hcan3, ARM_JOINT_ID_PLACEHOLDER, ARM_JOINT_ID_PLACEHOLDER, DM4340P}, // J2
    {&hcan3, ARM_JOINT_ID_PLACEHOLDER, ARM_JOINT_ID_PLACEHOLDER, DM4310},  // J3
    {&hcan3, ARM_JOINT_ID_PLACEHOLDER, ARM_JOINT_ID_PLACEHOLDER, DM4310},  // J4
    {&hcan3, ARM_JOINT_ID_PLACEHOLDER, ARM_JOINT_ID_PLACEHOLDER, DM4310},  // J5
};

/*
 * 关节换算参数, 全部是占位值, 上板前逐项标定:
 *   direction  电机正方向与MDH关节正方向相同为1, 相反为-1
 *   q_offset   电机零位(达妙上位机设置的零点)对应的MDH关节角, rad
 *   tau_max    关节总力矩上限, N·m. 首次上电取额定扭矩以下, 调好后再放开;
 *              超过电机MIT映射范围T_MAX的部分会在初始化时被截掉
 * 额定/峰值: DM6248P 30/97, DM4340P 12/40, DM4310 3.5/12.5 (N·m)
 */
static const ArmMpcJointConfig joint_cfg[ARM_JOINT_NUM] = {
    {.direction = 1.0f, .q_offset = 0.0f, .tau_max = 20.0f}, // J0 DM6248P
    {.direction = 1.0f, .q_offset = 0.0f, .tau_max = 8.0f},  // J1 DM4340P
    {.direction = 1.0f, .q_offset = 0.0f, .tau_max = 8.0f},  // J2 DM4340P
    {.direction = 1.0f, .q_offset = 0.0f, .tau_max = 3.0f},  // J3 DM4310
    {.direction = 1.0f, .q_offset = 0.0f, .tau_max = 3.0f},  // J4 DM4310
    {.direction = 1.0f, .q_offset = 0.0f, .tau_max = 3.0f},  // J5 DM4310
};

static DMMotorInstance *joint_motor[ARM_JOINT_NUM]; // 关节电机, 未注册的为NULL
static ArmMpcTask arm_mpc;                          // 机械臂控制实例

static Publisher_t *gimbal_pub;                   // 机械臂状态发布者(反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的机械臂状态
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

/* 检查同一总线上是否已有相同rx_id的电机, 重复注册会让CANRegister()死循环 */
static uint8_t JointCanIdConflict(uint8_t joint)
{
    for (uint8_t i = 0; i < joint; i++)
    {
        if (joint_motor[i] &&
            joint_motor_config[i].can_handle == joint_motor_config[joint].can_handle &&
            joint_motor_config[i].rx_id == joint_motor_config[joint].rx_id)
            return 1;
    }
    return 0;
}

static void JointMotorInit()
{
    for (uint8_t j = 0; j < ARM_JOINT_NUM; j++)
    {
        const Arm_Joint_Motor_Config_s *c = &joint_motor_config[j];
        if (c->tx_id == ARM_JOINT_ID_PLACEHOLDER)
        {
            LOGWARNING("[gimbal] joint [%d] CAN id not configured, motor not registered", j);
            continue;
        }
        if (JointCanIdConflict(j))
        {
            LOGERROR("[gimbal] joint [%d] rx id [%d] already used on this bus, motor not registered", j, c->rx_id);
            continue;
        }
        // 力控: 力矩直接由 ArmMpcTask 给出, 不用驱动里的PID
        // 方向统一在 joint_cfg 里处理, 电机的正反转标志保持 NORMAL
        Motor_Init_Config_s config = {
            .can_init_config = {
                .can_handle = c->can_handle,
                .tx_id = c->tx_id,
                .rx_id = c->rx_id,
            },
            .controller_setting_init_config = {
                .outer_loop_type = OPEN_LOOP,
                .close_loop_type = OPEN_LOOP,
                .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            },
            .motor_type = c->motor_type,
        };
        joint_motor[j] = DMMotorInit(&config);
    }
}

/* 把cmd消息转交给ArmMpcTask. 没收到新消息时模式和目标都保持上一帧 */
static void GimbalApplyCmd()
{
    ArmMpcTaskSetMode(&arm_mpc, gimbal_cmd_recv.mode == ARM_RUN ? ARM_MPC_MODE_RUN : ARM_MPC_MODE_ZERO_FORCE);
    ArmMpcTaskSetTarget(&arm_mpc, (ArmMpcMove)gimbal_cmd_recv.move, gimbal_cmd_recv.q_target, gimbal_cmd_recv.move_time);
}

static void GimbalSetFeedback()
{
    memcpy(gimbal_feedback_data.q, arm_mpc.q_actual, sizeof(gimbal_feedback_data.q));
    memcpy(gimbal_feedback_data.q_target, arm_mpc.q_target, sizeof(gimbal_feedback_data.q_target));
    gimbal_feedback_data.arrived = arm_mpc.arrived;
    gimbal_feedback_data.limp = arm_mpc.limp;
    gimbal_feedback_data.fault_flags = arm_mpc.fault_flags;
    gimbal_feedback_data.motor_online = arm_mpc.motor_online;
}

void GimbalInit()
{
    JointMotorInit();

    // 初始化后处于零力矩, cmd给出 ARM_RUN 后才会出力
    if (!ArmMpcTaskInit(&arm_mpc, joint_motor, joint_cfg))
        LOGERROR("[gimbal] ArmMpcTask init failed, check joint_cfg");

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/* 机械臂控制核心任务, 与RobotTask同频(INS_ARM_CONTROL_TICK_MS) */
void GimbalTask()
{
    // 获取机械臂控制数据
    if (SubGetMessage(gimbal_sub, &gimbal_cmd_recv))
        GimbalApplyCmd();

    // 反馈 -> 故障检测 -> 目标角 -> 轨迹 -> RNEA+MPC -> 电机力矩
    ArmMpcTaskRun(&arm_mpc);

    // 设置反馈数据并推送
    GimbalSetFeedback();
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
