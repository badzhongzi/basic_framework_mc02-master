/* 注意该文件应只用于任务初始化,只能被robot.c包含*/
#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "robot.h"
#include "ins_task.h"
#include "motor_task.h"
#include "referee27_task.h"
#include "master_process.h"
#include "daemon.h"
#include "HT04.h"
#include "buzzer.h"

#include "bsp_log.h"

osThreadId insTaskHandle;
osThreadId robotTaskHandle;
osThreadId motorTaskHandle;
osThreadId daemonTaskHandle;
osThreadId uiTaskHandle;

void StartINSTASK(void const *argument);
void StartMOTORTASK(void const *argument);
void StartDAEMONTASK(void const *argument);
void StartROBOTTASK(void const *argument);
void StartUITASK(void const *argument);

/**
 * @brief 初始化机器人任务,所有持续运行的任务都在这里初始化
 *
 */
// robottask 栈(4 KB)与 TCB 放入 480 MHz 零等待 DTCMRAM:
// 未开 D-Cache 时, mpc_cs_solve / rnea / crba 的局部数组全在任务栈上,
// 放 AXI SRAM 会导致单拍耗时 3.7 ms 超出 2 ms 预算. robottask 不把栈变量传给 DMA, 安全.
__attribute__((section(".dtcm"), aligned(8))) static uint32_t robot_task_stack[1024];
__attribute__((section(".dtcm"))) static osStaticThreadDef_t robot_task_tcb;

void OSTaskInit()
{
    // 【已禁用 INS 任务】INS_Init() 里 BMI088Init 是死等循环(无超时/无 osDelay),
    // 以 AboveNormal 优先级会阻塞所有 Normal 任务(含 DM 电机任务), 导致电机上电后
    // 要等 BMI088 初始化成功(数秒)才发使能帧、才变绿. 本项目不用 IMU, 直接不创建该任务.
    // VisionSend 已由 robot_cmd.c 的 RobotCMDTask 调用, 不会因此丢失.
    // osThreadDef(instask, StartINSTASK, osPriorityAboveNormal, 0, 1024);
    // insTaskHandle = osThreadCreate(osThread(instask), NULL);

    osThreadDef(motortask, StartMOTORTASK, osPriorityNormal, 0, 256);
    motorTaskHandle = osThreadCreate(osThread(motortask), NULL);

    osThreadDef(daemontask, StartDAEMONTASK, osPriorityNormal, 0, 128);
    daemonTaskHandle = osThreadCreate(osThread(daemontask), NULL);

    osThreadStaticDef(robottask, StartROBOTTASK, osPriorityNormal, 0, 1024,
                      robot_task_stack, &robot_task_tcb);
    robotTaskHandle = osThreadCreate(osThread(robottask), NULL);

    osThreadDef(uitask, StartUITASK, osPriorityNormal, 0, 512);
    uiTaskHandle = osThreadCreate(osThread(uitask), NULL);

    HTMotorControlInit(); // 没有注册HT电机则不会执行
}

__attribute__((noreturn)) void StartINSTASK(void const *argument)
{
    static float ins_start;
    static float ins_dt;
    uint32_t ins_wake_time;
    INS_Init(); // 确保BMI088被正确初始化.
    LOGINFO("[freeRTOS] INS Task Start");
    ins_wake_time = osKernelSysTick();
    for (;;)
    {
        // 1kHz
        ins_start = DWT_GetTimeline_ms();
        INS_Task();
        ins_dt = DWT_GetTimeline_ms() - ins_start;
        if (ins_dt > 1)
            LOGERROR("[freeRTOS] INS Task is being DELAY! dt = [%f]", &ins_dt);
        VisionSend(); // 解算完成后发送视觉数据,但是当前的实现不太优雅,后续若添加硬件触发需要重新考虑结构的组织
        osDelayUntil(&ins_wake_time, 1);
    }
}

__attribute__((noreturn)) void StartMOTORTASK(void const *argument)
{
    static float motor_dt;
    static float motor_start;
    uint32_t motor_wake_time;
    LOGINFO("[freeRTOS] MOTOR Task Start");
    motor_wake_time = osKernelSysTick();
    for (;;)
    {
        // 1kHz
        motor_start = DWT_GetTimeline_ms();
        MotorControlTask();
        motor_dt = DWT_GetTimeline_ms() - motor_start;
        if (motor_dt > 1)
            LOGERROR("[freeRTOS] MOTOR Task is being DELAY! dt = [%f]", &motor_dt);
        osDelayUntil(&motor_wake_time, 1);
    }
}

__attribute__((noreturn)) void StartDAEMONTASK(void const *argument)
{
    static float daemon_dt;
    static float daemon_start;
    uint32_t daemon_wake_time;
    BuzzerInit();
    LOGINFO("[freeRTOS] Daemon Task Start");
    daemon_wake_time = osKernelSysTick();
    for (;;)
    {
        // 100Hz
        daemon_start = DWT_GetTimeline_ms();
        DaemonTask();
        BuzzerTask();
        daemon_dt = DWT_GetTimeline_ms() - daemon_start;
        if (daemon_dt > 10)
            LOGERROR("[freeRTOS] Daemon Task is being DELAY! dt = [%f]", &daemon_dt);
        osDelayUntil(&daemon_wake_time, 10);
    }
}

__attribute__((noreturn)) void StartROBOTTASK(void const *argument)
{
    static float robot_dt;
    static float robot_start;
    uint32_t robot_wake_time;
    LOGINFO("[freeRTOS] ROBOT core Task Start");
    robot_wake_time = osKernelSysTick();
    for (;;)
    {
        // 500Hz
        robot_start = DWT_GetTimeline_ms();
        RobotTask();
        robot_dt = DWT_GetTimeline_ms() - robot_start;
        if (robot_dt > 5)
            LOGERROR("[freeRTOS] ROBOT core Task is being DELAY! dt = [%f]", &robot_dt);
        osDelayUntil(&robot_wake_time, 2);
    }
}

__attribute__((noreturn)) void StartUITASK(void const *argument)
{
    uint32_t ui_wake_time;
    LOGINFO("[freeRTOS] UI Task Start");
    R27MyUIInit();
    LOGINFO("[freeRTOS] UI Init Done, communication with ref has established");
    ui_wake_time = osKernelSysTick();
    for (;;)
    {
        // 1kHz
        // 每给裁判系统发送一包数据会挂起一次,详见 R27RefereeSendThrottled()
        R27UITask();
        osDelayUntil(&ui_wake_time, 1); // 即使没有任何UI需要刷新,也挂起一次,防止卡在UITask中无法切换
    }
}
