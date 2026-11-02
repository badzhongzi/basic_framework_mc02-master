/**
 * @file referee27_task.h
 * @brief 裁判系统 UI 绘制任务(2027 协议)
 */

#ifndef REFEREE27_TASK_H
#define REFEREE27_TASK_H

#include "rm_referee27.h"

/**
 * @brief 初始化裁判系统交互任务(UI 绘制 + 多机通信)
 *
 * @param referee_usart_handle 常规链路的串口句柄, C 板一般为 &huart6
 * @param UI_data              由上层应用填写的待显示数据(模式/功率等)
 * @return r27_referee_info_t* 裁判系统反馈数据, 失败返回 NULL
 * @note 内部会调用 R27RefereeInit(R27_LINK_COMMON, handle)
 */
r27_referee_info_t *R27UITaskInit(UART_HandleTypeDef *referee_usart_handle, r27_interactive_info_t *UI_data);

/**
 * @brief 使用已注册的链路实例初始化(需要同时使用图传/雷达链路时使用)
 */
r27_referee_info_t *R27UITaskInitWithLink(R27Link *link, r27_interactive_info_t *UI_data);

/**
 * @brief 在本任务开始前调用一次, 完成静态 UI 绘制(添加在 freertos.c 的 UITask 中)
 * @note 若未收到裁判系统数据(robot_id 为 0), 会阻塞等待; 未初始化则直接删除自身任务
 */
void R27MyUIInit(void);

/**
 * @brief UI 刷新任务, 10Hz 调用即可
 */
void R27UITask(void);

/**
 * @brief 获取本机 ID 信息(供其它模块发送交互数据时复用)
 */
r27_referee_id_t *R27GetRefereeId(void);

#endif /* REFEREE27_TASK_H */
