/**
 * @file referee27_task.c
 * @brief 裁判系统 UI 绘制任务(2027 协议)
 *
 * @note 移植自 modules/referee/referee_task.c, 差异:
 *       1. 使用 2027 协议的结构体/接口(R27_ 前缀);
 *       2. 去掉了上电后自动改变机器人模式的"联调自测"函数, 若需要请把
 *          R27_UI_SELF_TEST 置 1(会覆盖上层写入的模式, 仅供联调时确认链路正常);
 *       3. UI 频率由 R27_TX_MIN_INTERVAL_MS 统一节流(见 rm_referee27.h)。
 */

#include "referee27_task.h"
#include "referee27_UI.h"
#include "string.h"
#include "cmsis_os.h"
#include "bsp_log.h"

/* 联调自测: 置 1 后会周期性改变机器人模式并刷新 UI, 正式比赛必须为 0 */
#ifndef R27_UI_SELF_TEST
#define R27_UI_SELF_TEST 0
#endif

static r27_interactive_info_t *s_interactive; /* UI 绘制需要的机器人状态数据 */
static r27_referee_info_t *s_info;            /* 接收到的裁判系统数据 */
static r27_referee_id_t s_referee_id;         /* 本机 ID / 选手端 ID */

static void R27MyUIRefresh(void);
static void R27UIChangeCheck(void);
static void R27DeterminRobotID(void);
#if R27_UI_SELF_TEST
static void R27RobotModeTest(void);
#endif

/* ============================== 初始化 ============================== */

r27_referee_info_t *R27UITaskInitWithLink(R27Link *link, r27_interactive_info_t *UI_data)
{
    if (link == NULL)
        return NULL;

    s_info = &link->info;
    s_interactive = UI_data;
    s_info->init_flag = 1; /* 结构体已可用, 但 robot_id 仍需等待首帧 0x0201 */

    return s_info;
}

r27_referee_info_t *R27UITaskInit(UART_HandleTypeDef *referee_usart_handle, r27_interactive_info_t *UI_data)
{
    R27Link *link = R27RefereeInit(R27_LINK_COMMON, referee_usart_handle);
    if (link == NULL)
    {
        LOGERROR("[referee27_task] init common link failed");
        return NULL;
    }
    return R27UITaskInitWithLink(link, UI_data);
}

/**
 * @brief 根据 0x0201 中的 robot_id 确定颜色与选手端 ID
 */
static void R27DeterminRobotID(void)
{
    s_referee_id.Robot_ID = s_info->RobotStatus.robot_id;
    s_referee_id.Robot_Color = R27RobotColorOf(s_referee_id.Robot_ID);
    s_referee_id.Client_ID = R27ClientIdOf(s_referee_id.Robot_ID);
    s_referee_id.Receiver_Robot_ID = 0;
}

r27_referee_id_t *R27GetRefereeId(void)
{
    return &s_referee_id;
}

/* ============================== UI 绘制 ============================== */

static r27_interaction_figure_t s_shoot_line[10]; /* 射击准线 */
static r27_interaction_figure_t s_energy[3];      /* 电容能量条 */
static r27_interaction_string_t s_state_sta[6];   /* 机器人状态, 静态只画一次 */
static r27_interaction_string_t s_state_dyn[6];   /* 机器人状态, 动态需先 ADD 再 Change */
static uint32_t s_shoot_line_location[10] = {540, 960, 490, 515, 565};

void R27MyUIInit(void)
{
    if (s_info == NULL || s_interactive == NULL)
    {
        LOGERROR("[referee27_task] UI task init not called");
        vTaskDelete(NULL);
        return;
    }

    /* 等待首帧 0x0201(机器人性能体系数据), 其中才有 robot_id */
    while (s_info->RobotStatus.robot_id == 0)
        osDelay(100);

    R27DeterminRobotID();
    R27_UIDelete(&s_referee_id, R27_UI_Del_ALL, 0); /* 清空 UI */

    /* ---- 发射基准线 ---- */
    R27_UILineDraw(&s_shoot_line[0], "sl0", R27_UI_Graph_ADD, 7, R27_UI_Color_White, 3, 710, s_shoot_line_location[0], 1210, s_shoot_line_location[0]);
    R27_UILineDraw(&s_shoot_line[1], "sl1", R27_UI_Graph_ADD, 7, R27_UI_Color_White, 3, s_shoot_line_location[1], 340, s_shoot_line_location[1], 740);
    R27_UILineDraw(&s_shoot_line[2], "sl2", R27_UI_Graph_ADD, 7, R27_UI_Color_Yellow, 2, 810, s_shoot_line_location[2], 1110, s_shoot_line_location[2]);
    R27_UILineDraw(&s_shoot_line[3], "sl3", R27_UI_Graph_ADD, 7, R27_UI_Color_Yellow, 2, 810, s_shoot_line_location[3], 1110, s_shoot_line_location[3]);
    R27_UILineDraw(&s_shoot_line[4], "sl4", R27_UI_Graph_ADD, 7, R27_UI_Color_Yellow, 2, 810, s_shoot_line_location[4], 1110, s_shoot_line_location[4]);
    R27_UIGraphRefresh(&s_referee_id, 5, s_shoot_line[0], s_shoot_line[1], s_shoot_line[2], s_shoot_line[3], s_shoot_line[4]);

    /* ---- 车辆状态标签(静态) ---- */
    R27_UICharDraw(&s_state_sta[0], "ss0", R27_UI_Graph_ADD, 8, R27_UI_Color_Main, 15, 2, 150, 750, "chassis:");
    R27_UICharRefresh(&s_referee_id, s_state_sta[0]);
    R27_UICharDraw(&s_state_sta[1], "ss1", R27_UI_Graph_ADD, 8, R27_UI_Color_Yellow, 15, 2, 150, 700, "gimbal:");
    R27_UICharRefresh(&s_referee_id, s_state_sta[1]);
    R27_UICharDraw(&s_state_sta[2], "ss2", R27_UI_Graph_ADD, 8, R27_UI_Color_Orange, 15, 2, 150, 650, "shoot:");
    R27_UICharRefresh(&s_referee_id, s_state_sta[2]);
    R27_UICharDraw(&s_state_sta[3], "ss3", R27_UI_Graph_ADD, 8, R27_UI_Color_Pink, 15, 2, 150, 600, "frict:");
    R27_UICharRefresh(&s_referee_id, s_state_sta[3]);
    R27_UICharDraw(&s_state_sta[4], "ss4", R27_UI_Graph_ADD, 8, R27_UI_Color_Pink, 15, 2, 150, 550, "lid:");
    R27_UICharRefresh(&s_referee_id, s_state_sta[4]);

    /* ---- 车辆状态内容(动态) ----
     * 初始化时 xxx_last_mode 默认为 0, 因此初始内容应与 0 对应, 否则模式不变时不会触发刷新 */
    R27_UICharDraw(&s_state_dyn[0], "sd0", R27_UI_Graph_ADD, 8, R27_UI_Color_Main, 15, 2, 270, 750, "zeroforce");
    R27_UICharRefresh(&s_referee_id, s_state_dyn[0]);
    R27_UICharDraw(&s_state_dyn[1], "sd1", R27_UI_Graph_ADD, 8, R27_UI_Color_Yellow, 15, 2, 270, 700, "zeroforce");
    R27_UICharRefresh(&s_referee_id, s_state_dyn[1]);
    R27_UICharDraw(&s_state_dyn[2], "sd2", R27_UI_Graph_ADD, 8, R27_UI_Color_Orange, 15, 2, 270, 650, "off");
    R27_UICharRefresh(&s_referee_id, s_state_dyn[2]);
    R27_UICharDraw(&s_state_dyn[3], "sd3", R27_UI_Graph_ADD, 8, R27_UI_Color_Pink, 15, 2, 270, 600, "off");
    R27_UICharRefresh(&s_referee_id, s_state_dyn[3]);
    R27_UICharDraw(&s_state_dyn[4], "sd4", R27_UI_Graph_ADD, 8, R27_UI_Color_Pink, 15, 2, 270, 550, "open ");
    R27_UICharRefresh(&s_referee_id, s_state_dyn[4]);

    /* ---- 底盘功率(静态部分) ---- */
    R27_UICharDraw(&s_state_sta[5], "ss5", R27_UI_Graph_ADD, 7, R27_UI_Color_Green, 18, 2, 620, 230, "Power:");
    R27_UICharRefresh(&s_referee_id, s_state_sta[5]);

    /* 能量条外框 */
    R27_UIRectangleDraw(&s_energy[0], "ss6", R27_UI_Graph_ADD, 7, R27_UI_Color_Green, 2, 720, 140, 1220, 180);
    R27_UIGraphRefresh(&s_referee_id, 1, s_energy[0]);

    /* ---- 底盘功率(动态部分) ---- */
    R27_UIFloatDraw(&s_energy[1], "sd5", R27_UI_Graph_ADD, 8, R27_UI_Color_Green, 18, 2, 2, 750, 230, 24000);
    /* 能量条初始状态 */
    R27_UILineDraw(&s_energy[2], "sd6", R27_UI_Graph_ADD, 8, R27_UI_Color_Pink, 30, 720, 160, 1020, 160);
    R27_UIGraphRefresh(&s_referee_id, 2, s_energy[1], s_energy[2]);
}

/* ============================== 联调自测 ============================== */

#if R27_UI_SELF_TEST
static uint8_t s_test_count = 0;
static uint16_t s_test_stage = 0;

static void R27RobotModeTest(void)
{
    s_test_count++;
    if (s_test_count >= 50)
    {
        s_test_count = 0;
        s_test_stage++;
    }

    switch (s_test_stage % 4)
    {
    case 0:
        s_interactive->chassis_mode = CHASSIS_ZERO_FORCE;
        s_interactive->gimbal_mode = GIMBAL_ZERO_FORCE;
        s_interactive->shoot_mode = SHOOT_ON;
        s_interactive->friction_mode = FRICTION_ON;
        s_interactive->lid_mode = LID_OPEN;
        s_interactive->Chassis_Power_Data.chassis_power_mx += 3.5f;
        if (s_interactive->Chassis_Power_Data.chassis_power_mx >= 18.0f)
            s_interactive->Chassis_Power_Data.chassis_power_mx = 0.0f;
        break;
    case 1:
        s_interactive->chassis_mode = CHASSIS_ROTATE;
        s_interactive->gimbal_mode = GIMBAL_FREE_MODE;
        s_interactive->shoot_mode = SHOOT_OFF;
        s_interactive->friction_mode = FRICTION_OFF;
        s_interactive->lid_mode = LID_CLOSE;
        break;
    case 2:
        s_interactive->chassis_mode = CHASSIS_NO_FOLLOW;
        s_interactive->gimbal_mode = GIMBAL_GYRO_MODE;
        s_interactive->shoot_mode = SHOOT_ON;
        s_interactive->friction_mode = FRICTION_ON;
        s_interactive->lid_mode = LID_OPEN;
        break;
    case 3:
        s_interactive->chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        s_interactive->gimbal_mode = GIMBAL_ZERO_FORCE;
        s_interactive->shoot_mode = SHOOT_OFF;
        s_interactive->friction_mode = FRICTION_OFF;
        s_interactive->lid_mode = LID_CLOSE;
        break;
    default:
        break;
    }
}
#endif

/* ============================== 周期刷新 ============================== */

static void R27MyUIRefresh(void)
{
    R27UIChangeCheck();

    /* 底盘模式 */
    if (s_interactive->Interactive_Flag.chassis_flag == 1)
    {
        switch (s_interactive->chassis_mode)
        {
        case CHASSIS_ZERO_FORCE:
            R27_UICharDraw(&s_state_dyn[0], "sd0", R27_UI_Graph_Change, 8, R27_UI_Color_Main, 15, 2, 270, 750, "zeroforce");
            break;
        case CHASSIS_ROTATE:
            R27_UICharDraw(&s_state_dyn[0], "sd0", R27_UI_Graph_Change, 8, R27_UI_Color_Main, 15, 2, 270, 750, "rotate   ");
            break;
        case CHASSIS_NO_FOLLOW:
            R27_UICharDraw(&s_state_dyn[0], "sd0", R27_UI_Graph_Change, 8, R27_UI_Color_Main, 15, 2, 270, 750, "nofollow ");
            break;
        case CHASSIS_FOLLOW_GIMBAL_YAW:
            R27_UICharDraw(&s_state_dyn[0], "sd0", R27_UI_Graph_Change, 8, R27_UI_Color_Main, 15, 2, 270, 750, "follow   ");
            break;
        default:
            break;
        }
        R27_UICharRefresh(&s_referee_id, s_state_dyn[0]);
        s_interactive->Interactive_Flag.chassis_flag = 0;
    }

    /* 云台模式 */
    if (s_interactive->Interactive_Flag.gimbal_flag == 1)
    {
        switch (s_interactive->gimbal_mode)
        {
        case GIMBAL_ZERO_FORCE:
            R27_UICharDraw(&s_state_dyn[1], "sd1", R27_UI_Graph_Change, 8, R27_UI_Color_Yellow, 15, 2, 270, 700, "zeroforce");
            break;
        case GIMBAL_FREE_MODE:
            R27_UICharDraw(&s_state_dyn[1], "sd1", R27_UI_Graph_Change, 8, R27_UI_Color_Yellow, 15, 2, 270, 700, "free     ");
            break;
        case GIMBAL_GYRO_MODE:
            R27_UICharDraw(&s_state_dyn[1], "sd1", R27_UI_Graph_Change, 8, R27_UI_Color_Yellow, 15, 2, 270, 700, "gyro     ");
            break;
        default:
            break;
        }
        R27_UICharRefresh(&s_referee_id, s_state_dyn[1]);
        s_interactive->Interactive_Flag.gimbal_flag = 0;
    }

    /* 发射模式 */
    if (s_interactive->Interactive_Flag.shoot_flag == 1)
    {
        R27_UICharDraw(&s_state_dyn[2], "sd2", R27_UI_Graph_Change, 8, R27_UI_Color_Pink, 15, 2, 270, 650,
                       s_interactive->shoot_mode == SHOOT_ON ? "on " : "off");
        R27_UICharRefresh(&s_referee_id, s_state_dyn[2]);
        s_interactive->Interactive_Flag.shoot_flag = 0;
    }

    /* 摩擦轮 */
    if (s_interactive->Interactive_Flag.friction_flag == 1)
    {
        R27_UICharDraw(&s_state_dyn[3], "sd3", R27_UI_Graph_Change, 8, R27_UI_Color_Pink, 15, 2, 270, 600,
                       s_interactive->friction_mode == FRICTION_ON ? "on " : "off");
        R27_UICharRefresh(&s_referee_id, s_state_dyn[3]);
        s_interactive->Interactive_Flag.friction_flag = 0;
    }

    /* 弹舱盖 */
    if (s_interactive->Interactive_Flag.lid_flag == 1)
    {
        R27_UICharDraw(&s_state_dyn[4], "sd4", R27_UI_Graph_Change, 8, R27_UI_Color_Pink, 15, 2, 270, 550,
                       s_interactive->lid_mode == LID_OPEN ? "open " : "close");
        R27_UICharRefresh(&s_referee_id, s_state_dyn[4]);
        s_interactive->Interactive_Flag.lid_flag = 0;
    }

    /* 底盘功率/能量条 */
    if (s_interactive->Interactive_Flag.Power_flag == 1)
    {
        R27_UIFloatDraw(&s_energy[1], "sd5", R27_UI_Graph_Change, 8, R27_UI_Color_Green, 18, 2, 2, 750, 230,
                        (int32_t)(s_interactive->Chassis_Power_Data.chassis_power_mx * 1000));
        R27_UILineDraw(&s_energy[2], "sd6", R27_UI_Graph_Change, 8, R27_UI_Color_Pink, 30, 720, 160,
                       (uint32_t)(750 + s_interactive->Chassis_Power_Data.chassis_power_mx * 30), 160);
        R27_UIGraphRefresh(&s_referee_id, 2, s_energy[1], s_energy[2]);
        s_interactive->Interactive_Flag.Power_flag = 0;
    }
}

/**
 * @brief 模式切换检测, 模式发生变化时置位对应 flag
 */
static void R27UIChangeCheck(void)
{
    if (s_interactive->chassis_mode != s_interactive->chassis_last_mode)
    {
        s_interactive->Interactive_Flag.chassis_flag = 1;
        s_interactive->chassis_last_mode = s_interactive->chassis_mode;
    }

    if (s_interactive->gimbal_mode != s_interactive->gimbal_last_mode)
    {
        s_interactive->Interactive_Flag.gimbal_flag = 1;
        s_interactive->gimbal_last_mode = s_interactive->gimbal_mode;
    }

    if (s_interactive->shoot_mode != s_interactive->shoot_last_mode)
    {
        s_interactive->Interactive_Flag.shoot_flag = 1;
        s_interactive->shoot_last_mode = s_interactive->shoot_mode;
    }

    if (s_interactive->friction_mode != s_interactive->friction_last_mode)
    {
        s_interactive->Interactive_Flag.friction_flag = 1;
        s_interactive->friction_last_mode = s_interactive->friction_mode;
    }

    if (s_interactive->lid_mode != s_interactive->lid_last_mode)
    {
        s_interactive->Interactive_Flag.lid_flag = 1;
        s_interactive->lid_last_mode = s_interactive->lid_mode;
    }

    if (s_interactive->Chassis_Power_Data.chassis_power_mx != s_interactive->Chassis_last_Power_Data.chassis_power_mx)
    {
        s_interactive->Interactive_Flag.Power_flag = 1;
        s_interactive->Chassis_last_Power_Data.chassis_power_mx = s_interactive->Chassis_Power_Data.chassis_power_mx;
    }
}

void R27UITask(void)
{
    if (s_info == NULL || s_interactive == NULL)
        return;

#if R27_UI_SELF_TEST
    R27RobotModeTest();
#endif

    R27MyUIRefresh();
}
