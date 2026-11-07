// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "bmi088.h"
#include "flysky.h"
#include "master_link.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include <math.h>
#include <string.h>

_Static_assert(ARM_JOINT_NUM == MASTER_LINK_JOINT_NUM, "joint count mismatch");

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息(本项目不用, 保持全0)
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息

#ifdef RC_USE_DJI
static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
#endif
#ifdef RC_USE_FLYSKY
static FS_ctrl_t *fs_data;              // 富斯遥控器数据, [0] 是当前帧
#endif
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
static Vision_Send_s vision_send_data;  // 视觉发送数据

static Publisher_t *gimbal_cmd_pub;            // 机械臂控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 机械臂反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给机械臂的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从机械臂获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息(本项目不用, 保持全0)
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态

BMI088Instance *bmi088_test; // 云台IMU
BMI088_Data_t bmi088_data;
void RobotCMDInit()
{
#ifdef RC_USE_DJI
    rc_data = RemoteControlInit(&huart5);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
#endif
#ifdef RC_USE_FLYSKY
    fs_data = FSControlInit(&huart5);       // 富斯接收机 SBUS, 和 DBUS 用同一个带反相器的串口
#endif
    MasterLinkRxInit(&MASTER_LINK_UART);    // 主臂(同构遥控器)链路
    vision_recv_data = VisionInit(&huart9); // 视觉通信串口

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif // GIMBAL_BOARD
    gimbal_cmd_send.mode = ARM_ZERO_FORCE; // 上电不出力, 遥控器给出运行模式后才出力

#ifdef RC_USE_DJI
    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
#endif
#ifdef RC_USE_FLYSKY
    // 上电锁在急停: 必须先把 switch_l1 拨到上(确认)一次才能出力, 避免上电时开关已在运行位就直接出力
    robot_state = ROBOT_STOP;
#endif
}

/* ------------------------------ 机械臂基础接口 ------------------------------ */
/*
 * 二次开发只需要用下面几个函数, 不用直接改 gimbal_cmd_send:
 *   ArmTarget()     当前目标角(每周期开头同步成gimbal实际生效的目标), 可以直接在上面累加增量
 *   ArmFollow(q)    跟随目标角q, 固定100ms平滑, 每帧都可以调(遥控器)
 *   ArmMoveTo(q,t)  用t秒走到q(点位/自动流程)
 *   ArmIsArrived()  是否已停在目标角
 *   ArmZeroForce()  零力矩(同时是故障确认); ArmRun() 恢复出力
 * 零力矩期间目标角自动跟随实际角, 切回运行时机械臂停在原地.
 */
static float arm_target[ARM_JOINT_NUM]; // 本周期要发给gimbal的目标角, rad

/* 每周期开头调用: 从gimbal实际生效的目标接着改, 超出限位的部分不会累积, 零力矩下就是实际角 */
static void ArmSyncTarget()
{
    memcpy(arm_target, gimbal_fetch_data.q_target, sizeof(arm_target));
}

static float *ArmTarget()
{
    return arm_target;
}

static void ArmFollow(const float q[ARM_JOINT_NUM])
{
    memmove(arm_target, q, sizeof(arm_target));
    gimbal_cmd_send.move = ARM_MOVE_FOLLOW;
}

static void ArmMoveTo(const float q[ARM_JOINT_NUM], float move_time)
{
    memmove(arm_target, q, sizeof(arm_target));
    gimbal_cmd_send.move = ARM_MOVE_TIMED;
    gimbal_cmd_send.move_time = move_time;
}

/* 注意: 目标刚改的那一周期读到的还是上一个目标的结果, 下一周期才准 */
static uint8_t ArmIsArrived()
{
    return gimbal_fetch_data.arrived && !gimbal_fetch_data.limp;
}

static void ArmZeroForce()
{
    gimbal_cmd_send.mode = ARM_ZERO_FORCE;
}

static void ArmRun()
{
    gimbal_cmd_send.mode = ARM_RUN;
}

/* ------------------------------ 自动流程示例 ------------------------------ */
/*
 * 自动流程 = 一张点位表, 一步走完(arrived)再走下一步. 关节角都是占位值(URDF零位),
 * 实物标定关节限位之后再改.
 */
typedef struct
{
    float q[ARM_JOINT_NUM]; // 目标角, rad
    float move_time;        // 走到这个点的时长, s
} Arm_Step_s;

/* unused: 富斯模式下还没有触发自动流程的按键 */
__attribute__((unused)) static const Arm_Step_s arm_seq_home[] = {
    {{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, 2.0f},
};

static const Arm_Step_s *arm_seq;  // 正在执行的点位表, NULL表示没有
static uint8_t arm_seq_len;        // 点位数
static uint8_t arm_seq_step;       // 当前步
static uint8_t arm_seq_step_sent;  // 当前步的目标是否已经下发

__attribute__((unused)) static void ArmSeqStart(const Arm_Step_s *steps, uint8_t len)
{
    arm_seq = steps;
    arm_seq_len = len;
    arm_seq_step = 0;
    arm_seq_step_sent = 0;
}

static void ArmSeqStop()
{
    arm_seq = NULL;
}

/* 每周期调用一次 */
static void ArmSeqRun()
{
    if (arm_seq == NULL)
        return;
    if (gimbal_fetch_data.limp) // 零力矩/故障时流程作废, 恢复后不会接着走
    {
        ArmSeqStop();
        return;
    }
    // 每周期都给同一个目标, gimbal只在目标变化时规划一次
    ArmMoveTo(arm_seq[arm_seq_step].q, arm_seq[arm_seq_step].move_time);
    if (!arm_seq_step_sent) // 刚下发的这一周期, 反馈还是上一个目标的, 下一周期再判断到位
    {
        arm_seq_step_sent = 1;
        return;
    }
    if (ArmIsArrived())
    {
        arm_seq_step_sent = 0;
        if (++arm_seq_step >= arm_seq_len)
            ArmSeqStop();
    }
}

/* ------------------------------ 遥控器示例 ------------------------------ */
#ifdef RC_USE_DJI

// 增量式遥控: 摇杆满量程(660)对应的关节速度, rad/s; 死区防止摇杆回中不准导致漂移
#define ARM_RC_FULL_SPEED 1.0f
#define ARM_RC_DEADBAND 20

static uint32_t rc_dwt_cnt; // 算两次遥控器控制之间的时间
static int16_t last_dial;   // 上一周期的拨轮值, 用于检测拨轮拨到底的边沿
static uint16_t last_key_g_count;
static uint16_t last_key_b_count;

/* 摇杆值(-660~660)转成关节速度, rad/s */
static float RockerToSpeed(int16_t rocker)
{
    if (rocker < ARM_RC_DEADBAND && rocker > -ARM_RC_DEADBAND)
        return 0.0f;
    return ARM_RC_FULL_SPEED * (float)rocker / 660.0f;
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *        增量式示例: 摇杆给关节速度, 乘以时间累加到目标角上
 */
static void RemoteControlSet()
{
    float speed[ARM_JOINT_NUM] = {0};
    float dt = DWT_GetDeltaT(&rc_dwt_cnt);
    uint8_t moving = 0;

    if (dt > 0.02f) // 刚切进遥控器控制或任务被卡住过, 防止积出一大步
        dt = 0.0f;

    if (switch_is_down(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[下], 四个摇杆通道控制J0~J3
    {
        speed[0] = RockerToSpeed(rc_data[TEMP].rc.rocker_l_);
        speed[1] = RockerToSpeed(rc_data[TEMP].rc.rocker_l1);
        speed[2] = RockerToSpeed(rc_data[TEMP].rc.rocker_r1);
        speed[3] = RockerToSpeed(rc_data[TEMP].rc.rocker_r_);
    }
    else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[中], 左摇杆控制J4~J5
    {
        speed[4] = RockerToSpeed(rc_data[TEMP].rc.rocker_l_);
        speed[5] = RockerToSpeed(rc_data[TEMP].rc.rocker_l1);
    }
    ArmRun();

    for (uint8_t j = 0; j < ARM_JOINT_NUM; j++)
        if (speed[j] != 0.0f)
            moving = 1;
    if (moving) // 推杆打断正在走的自动流程
    {
        float *q = ArmTarget();
        ArmSeqStop();
        // 之前在走定时点位: 目标是远处的终点, 从当前实际角开始累加, 否则会接着冲向那个终点
        if (gimbal_cmd_send.move != ARM_MOVE_FOLLOW)
            memcpy(q, gimbal_fetch_data.q, sizeof(gimbal_fetch_data.q));
        for (uint8_t j = 0; j < ARM_JOINT_NUM; j++)
            q[j] += speed[j] * dt;
        ArmFollow(q);
    }

    // 拨轮向上拨到底(边沿触发), 走预设点位
    if (rc_data[TEMP].rc.dial < -500 && last_dial >= -500)
        ArmSeqStart(arm_seq_home, sizeof(arm_seq_home) / sizeof(arm_seq_home[0]));
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    // G键走预设点位, B键停在当前位置
    ArmRun();
    if (rc_data[TEMP].key_count[KEY_PRESS][Key_G] != last_key_g_count)
        ArmSeqStart(arm_seq_home, sizeof(arm_seq_home) / sizeof(arm_seq_home[0]));
    else if (rc_data[TEMP].key_count[KEY_PRESS][Key_B] != last_key_b_count)
    {
        ArmSeqStop();
        ArmFollow(gimbal_fetch_data.q);
    }
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    // 拨轮的向下拨超过一半进入急停模式.注意向打时下拨轮是正
    if (rc_data[TEMP].rc.dial > 300 || robot_state == ROBOT_STOP) // 还需添加重要应用和模块离线的判断
    {
        robot_state = ROBOT_STOP;
        ArmSeqStop();
        ArmZeroForce(); // 机械臂失力会下落, 注意托住
        LOGERROR("[CMD] emergency stop!");
    }
    // 遥控器右侧开关为[上],恢复正常运行
    if (switch_is_up(rc_data[TEMP].rc.switch_right))
    {
        robot_state = ROBOT_READY;
        LOGINFO("[CMD] reinstate, robot ready");
    }
}

/*
 * DJI 遥控器的每周期入口. 以后换回 DJI 遥控器时, 主臂控制按富斯版 FlySkyCmd() 的写法
 * 加一个分支调 MasterArmControl() 即可 (开关分配到时再定).
 */
static void DjiCmd()
{
    // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠
    if (switch_is_down(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[下],遥控器控制
        RemoteControlSet();
    else if (switch_is_up(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[上],键盘控制
        MouseKeySet();
    ArmSeqRun();

    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    last_dial = rc_data[TEMP].rc.dial;
    last_key_g_count = rc_data[TEMP].key_count[KEY_PRESS][Key_G];
    last_key_b_count = rc_data[TEMP].key_count[KEY_PRESS][Key_B];
}
#endif // RC_USE_DJI

/* ------------------------------ 主臂(同构遥控器)控制 ------------------------------ */
/*
 * 主臂经串口(modules/master_link)发来 6 个关节角, 已经是从臂的 URDF 关节角.
 * 接管流程: 取一次主臂角度的快照 -> ArmMoveTo 慢速对齐 -> 到位且主臂没动 -> ArmFollow 跟随.
 * 链路不可用(掉线/心跳停/主端报告无效)时置 ROBOT_STOP 锁存, 和急停一样要人工确认才能恢复.
 */
#define ARM_ALIGN_SPEED 0.3f    // 对齐时按最大关节差/这个速度估时长, rad/s
#define ARM_ALIGN_TIME_MIN 1.0f // s
#define ARM_ALIGN_TIME_MAX 5.0f // s, 和 gimbal 侧 TIMED 时长上限一致
#define ARM_ALIGN_TOL 0.05f     // 对齐完成时主臂离快照的最大偏差, rad; 超过就重新对齐

typedef enum
{
    MASTER_IDLE = 0, // 下一周期取快照开始对齐
    MASTER_ALIGN,    // 正在慢速走向快照
    MASTER_FOLLOW,   // 跟随主臂
} Master_Ctrl_State_e;

static Master_Ctrl_State_e master_state;
static float master_align_q[ARM_JOINT_NUM]; // 对齐目标(主臂快照, 已夹进限位)
static float master_align_time;
static uint8_t master_align_sent;     // 对齐目标已下发一个周期(arrived 晚一周期)
static uint32_t master_link_lost_cnt; // 主臂控制时链路不可用导致急停的次数, 调试看

static void ClampToJointLimit(float q[ARM_JOINT_NUM])
{
    for (uint8_t j = 0; j < ARM_JOINT_NUM; j++)
    {
        if (q[j] < ARM_JOINT_Q_MIN)
            q[j] = ARM_JOINT_Q_MIN;
        else if (q[j] > ARM_JOINT_Q_MAX)
            q[j] = ARM_JOINT_Q_MAX;
    }
}

static float MaxAbsDiff(const float a[ARM_JOINT_NUM], const float b[ARM_JOINT_NUM])
{
    float m = 0.0f;
    for (uint8_t j = 0; j < ARM_JOINT_NUM; j++)
        if (fabsf(a[j] - b[j]) > m)
            m = fabsf(a[j] - b[j]);
    return m;
}

/* 反馈的实际角. gimbal_fetch_data 是 pack(1) 的, 不能直接把成员地址当 float* 用 (文档 3.3.5) */
static void ArmActualQ(float q[ARM_JOINT_NUM])
{
    memcpy(q, gimbal_fetch_data.q, sizeof(gimbal_fetch_data.q));
}

static void MasterReset()
{
    master_state = MASTER_IDLE;
}
/**
 * @brief 主臂控制, 每周期调用. 链路不可用时置 ROBOT_STOP (调用者负责之后发零力矩)
 */
__attribute__((unused)) static void MasterArmControl()
{
    float q_m[ARM_JOINT_NUM];
    float q_act[ARM_JOINT_NUM];

    ArmSeqStop();
    if (!MasterLinkIsValid())
    {
        master_link_lost_cnt++;
        robot_state = ROBOT_STOP; // 锁存, 恢复要人工确认, 确认后重新对齐
        MasterReset();
        return;
    }
    MasterLinkGetQ(q_m);
    ClampToJointLimit(q_m); // gimbal 也会夹; 这里先夹是为了对齐误差的比较能收敛
    ArmRun();

    // 刚切 RUN 的那一周期反馈还是 limp, 或者 gimbal 有锁存故障: 目标不会生效, 等 limp 解除再开始对齐
    if (gimbal_fetch_data.limp)
    {
        MasterReset();
        return;
    }

    switch (master_state)
    {
    case MASTER_IDLE:
        memcpy(master_align_q, q_m, sizeof(master_align_q));
        ArmActualQ(q_act);
        master_align_time = MaxAbsDiff(master_align_q, q_act) / ARM_ALIGN_SPEED;
        if (master_align_time < ARM_ALIGN_TIME_MIN)
            master_align_time = ARM_ALIGN_TIME_MIN;
        if (master_align_time > ARM_ALIGN_TIME_MAX)
            master_align_time = ARM_ALIGN_TIME_MAX;
        master_align_sent = 0;
        master_state = MASTER_ALIGN;
        /* fall through */
    case MASTER_ALIGN:
        ArmMoveTo(master_align_q, master_align_time); // 快照固定, gimbal 只规划一次
        if (!master_align_sent) // 刚下发的这一周期, arrived 还是上一个目标的
        {
            master_align_sent = 1;
            break;
        }
        if (ArmIsArrived())
        {
            if (MaxAbsDiff(q_m, master_align_q) < ARM_ALIGN_TOL)
            {
                master_state = MASTER_FOLLOW;
                ArmFollow(q_m);
            }
            else
                master_state = MASTER_IDLE; // 对齐期间主臂被动过, 下一周期重新取快照
        }
        break;
    case MASTER_FOLLOW:
        ArmFollow(q_m);
        break;
    }
}
/* ------------------------------ 富斯遥控器 (调试用) ------------------------------ */
#ifdef RC_USE_FLYSKY
/*
 * 开关分配:
 *   switch_l1 (最左拨杆)  上: 零力矩, 同时是急停/故障的确认; 中、下: 允许出力
 *   switch_r2 (最右拨杆)  下: 主臂控制; 其他: 富斯摇杆控制关节
 *   switch_l2 (左侧第2个) 富斯控制时选关节组: 上 -> J0~J2, 其他 -> J3~J5
 * 摇杆只用会自动回中的三个通道 (右水平 rocker_r_ -> 组内第1个关节, 右竖直 rocker_r1 -> 第2个,
 * 左水平 rocker_l_ -> 第3个). 左竖直 rocker_l1 一般是油门杆, 不回中, 不用来控制关节.
 * 急停锁存 (ROBOT_STOP) 的来源: 上电、遥控器离线、主臂控制时链路不可用. 恢复: switch_l1 拨到上.
 */
#define FS_ROCKER_FULL 784       // SBUS 摇杆满量程
#define FS_ROCKER_DEADBAND 30
#define FS_ARM_FULL_SPEED 1.0f   // 满量程对应的关节速度, rad/s, 不要超过 dq_max(3)

typedef enum
{
    CTRL_SRC_NONE = 0,
    CTRL_SRC_FLYSKY,
    CTRL_SRC_MASTER,
} Ctrl_Source_e;

static Ctrl_Source_e ctrl_source; // 上一周期谁在控制
static uint32_t fs_dwt_cnt;

static float FsRockerToSpeed(int16_t rocker)
{
    if (rocker < FS_ROCKER_DEADBAND && rocker > -FS_ROCKER_DEADBAND)
        return 0.0f;
    return FS_ARM_FULL_SPEED * (float)rocker / (float)FS_ROCKER_FULL;
}

/* 富斯摇杆点动关节, 写法同 DJI 版 RemoteControlSet() */
static void FlySkyJointControl()
{
    float speed[ARM_JOINT_NUM] = {0};
    float dt = DWT_GetDeltaT(&fs_dwt_cnt);
    uint8_t base = fs_data[TEMP].switch_l2 == FS_SW_UP ? 0 : 3;
    uint8_t moving = 0;

    if (dt > 0.02f) // 刚切进来或任务被卡住过, 防止积出一大步
        dt = 0.0f;
    speed[base + 0] = FsRockerToSpeed(fs_data[TEMP].rocker_r_);
    speed[base + 1] = FsRockerToSpeed(fs_data[TEMP].rocker_r1);
    speed[base + 2] = FsRockerToSpeed(fs_data[TEMP].rocker_l_);
    ArmRun();

    for (uint8_t j = 0; j < ARM_JOINT_NUM; j++)
        if (speed[j] != 0.0f)
            moving = 1;
    if (!moving)
        return;
    float *q = ArmTarget();
    ArmSeqStop();
    if (gimbal_cmd_send.move != ARM_MOVE_FOLLOW) // 之前在走定时(对齐/点位), 从实际角开始累加
        ArmActualQ(q);
    for (uint8_t j = 0; j < ARM_JOINT_NUM; j++)
        q[j] += speed[j] * dt;
    ArmFollow(q);
}
/* 富斯遥控器的每周期入口: 急停/确认 -> 选控制源 */
static void FlySkyCmd()
{
    if (!FSControlIsOnline()) // 离线时开关值是 0, 不能当成任何位置
        robot_state = ROBOT_STOP;
    else if (fs_data[TEMP].switch_l1 == FS_SW_UP) // 确认位: 解除锁存, 但这一位本身就是零力矩
    {
        robot_state = ROBOT_READY;
        ArmSeqStop();
        ArmZeroForce(); // 也是 gimbal 故障锁存的确认 (至少一个周期 ZERO_FORCE)
        MasterReset();
        ctrl_source = CTRL_SRC_NONE;
        return;
    }

    if (robot_state == ROBOT_READY)
    {
        if (fs_data[TEMP].switch_r2 == FS_SW_DOWN)
        {
            MasterArmControl(); // 链路不可用时会把 robot_state 置成 STOP
            ctrl_source = CTRL_SRC_MASTER;
        }
        else
        {
            // 刚从主臂控制切过来: 原地停住, 不要接着走向主臂最后的角度(或对齐目标)
            if (ctrl_source == CTRL_SRC_MASTER && !gimbal_fetch_data.limp)
            {
                float *q = ArmTarget();
                ArmActualQ(q);
                ArmFollow(q);
            }
            MasterReset();
            FlySkyJointControl();
            ArmSeqRun(); // 富斯模式下还没有启动流程的按键, 这里保持和 DJI 版相同的调用位置
            ctrl_source = CTRL_SRC_FLYSKY;
        }
    }

    if (robot_state == ROBOT_STOP) // 锁存, 直到 switch_l1 拨到上
    {
        ArmSeqStop();
        ArmZeroForce(); // 机械臂失力会下落, 注意托住
        MasterReset();
        ctrl_source = CTRL_SRC_NONE;
    }
}
#endif // RC_USE_FLYSKY

/* 机器人核心控制任务,与RobotTask同频运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
   // BMI088Acquire(bmi088_test,&bmi088_data) ;
    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // 目标角默认保持gimbal当前生效的目标, 下面的控制逻辑在它上面修改
    ArmSyncTarget();

#ifdef RC_USE_DJI
    DjiCmd();
#endif
#ifdef RC_USE_FLYSKY
    FlySkyCmd();
#endif

    memcpy(gimbal_cmd_send.q_target, arm_target, sizeof(gimbal_cmd_send.q_target));

    // 设置视觉发送数据,还需增加加速度和角速度数据
    // VisionSetFlag(chassis_fetch_data.enemy_color,,chassis_fetch_data.bullet_speed)

    // 推送消息,双板通信,视觉通信等
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    VisionSend(&vision_send_data);
}
