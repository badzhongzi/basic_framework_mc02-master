/**
 * @file flysky.h
 * @brief [iBUS模式，波特率115200，数据位8位，停止位1位，无奇偶校验
 * SBUS模式(反相)，波特率100_000，数据位9位，停止位2位，偶检验]
 * @author Univs_he
 * @date 2025-12-15
 */

#ifndef FLY_SKY_H
#define FLY_SKY_H

#include <stdint.h>
#include "main.h"
#include "usart.h"

#define FS_SBUS
// #define FS_iBUS

/* User configuration */

#define IBUS_LENGTH				0x20	// 32 bytes
#define IBUS_COMMAND40			0x40	// Command to set servo or motor speed is always 0x40

/* ----------------------- RC Switch Definition----------------------------- */
#if defined(FS_SBUS) && !defined(FS_iBUS)
#define FS_DATA_UP ((uint16_t)240)   // 开关向上时的值
#define FS_DATA_MID ((uint16_t)1024)  // 开关中间时的值
#define FS_DATA_DOWN ((uint16_t)1807) // 开关向下时的值

#define FS_DATA_MAX ((uint16_t)1807) // 最小值
#define FS_DATA_MIN ((uint16_t)240) // 最大值
#endif

#if defined(FS_iBUS) && !defined(FS_SBUS)
#define FS_DATA_UP ((uint16_t)1000)   // 开关向上时的值
#define FS_DATA_MID ((uint16_t)1500)  // 开关中间时的值
#define FS_DATA_DOWN ((uint16_t)2000) // 开关向下时的值

#define FS_DATA_MAX ((uint16_t)2000) // 最小值
#define FS_DATA_MIN ((uint16_t)1000) // 最大值
#endif
/* ----------------------- RC Switch Definition----------------------------- */
#define FS_SW_UP ((uint16_t)1)   // 开关向上时的值
#define FS_SW_MID ((uint16_t)3)  // 开关中间时的值
#define FS_SW_DOWN ((uint16_t)2) // 开关向下时的值


#define fs_data_dead_limit(data, dealine)        			\
    {                                                    	\
        if((data) > (dealine) || (data) < -(dealine)) 		\
        {                                                	\
            (data) = (data);                          		\
        }                                                	\
        else                                             	\
        {                                                	\
            (data) = 0;                                		\
        }                                                	\
    }
	

#define fs_data_change(data)                                \
	{                                                    	\
		switch(data)										\
		{													\
			case FS_DATA_DOWN:							    \
				data = FS_SW_DOWN;							\
				break;										\
			case FS_DATA_MID:								\
				data = FS_SW_MID;							\
				break;										\
			case FS_DATA_UP:								\
				data = FS_SW_UP;							\
				break;										\
			default:										\
				break;										\
		}													\
	}

/* ----------------------- Data Struct ------------------------------------- */
typedef struct
{
    int16_t rocker_r_;      // 左水平，SBUS[-784~783],iBUS[-500~500]
    int16_t rocker_r1;      // 左竖直，SBUS[-784~783],iBUS[-500~500]
    int16_t rocker_l1;      // 右水平，SBUS[-784~783],iBUS[-500~500]
    int16_t rocker_l_;      // 右竖直，SBUS[-784~783],iBUS[-500~500]

    uint16_t switch_l1;  // 最左侧拨杆
    uint16_t switch_l2;  // 左侧第2个拨杆
    uint16_t switch_r1;  // 右侧第1个拨杆
    uint16_t switch_r2;  // 最右侧拨杆

    int16_t knob_l;         // 左边旋钮，SBUS[0~1567],iBUS[0~1000]
    int16_t knob_r;         // 右边旋钮，SBUS[0~1567],iBUS[0~1000]

} FS_ctrl_t;

/* ------------------------- Internal Data ----------------------------------- */

/**
 * @brief 初始化遥控器,该函数会将遥控器注册到串口
 *
 * @attention 注意分配正确的串口硬件,遥控器在C板上使用USART3
 *
 */
FS_ctrl_t *FSControlInit(UART_HandleTypeDef *fs_usart_handle);

/**
 * @brief 检查遥控器是否在线,若尚未初始化也视为离线
 *
 * @return uint8_t 1:在线 0:离线
 */
uint8_t FSControlIsOnline();

#endif
