#include "flysky.h"
#include "string.h"
#include "bsp_usart.h"
#include "memory.h"
#include "stdlib.h"
#include "daemon.h"
#include "bsp_log.h"


#define IBUS_USER_CHANNELS		10
#define SBUS_USER_CHANNELS		10

#if defined(FS_SBUS) && !defined(FS_iBUS)
    #define FS_FRAME_SIZE 25u // 遥控器接收的buffer大小
#endif

#if defined(FS_iBUS) && !defined(FS_SBUS)
    #define FS_FRAME_SIZE 32u // 遥控器接收的buffer大小
#endif
// 用于遥控器数据读取,遥控器数据是一个大小为2的数组
#define LAST 1
#define TEMP 0

// 遥控器数据
static FS_ctrl_t fs_ctrl[2];
static uint8_t fs_init_flag = 0;

// 遥控器拥有的串口实例,因为遥控器是单例,所以这里只有一个,就不封装了
static USARTInstance *fs_usart_instance;
static DaemonInstance *fs_daemon_instance;

/**
 * @brief 矫正遥控器摇杆的值,超过660或者小于-660的值都认为是无效值,置0
 *
 */
static void RectifyRCjoystick()
{
    for (uint8_t i = 0; i < 4; ++i)
    {
        if (abs(*(&fs_ctrl[TEMP].rocker_r_ + i)) > FS_DATA_MAX)
            *(&fs_ctrl[TEMP].rocker_r_ + i) = 0;
    }

}

static uint8_t fs_rx_sta=0;
/**
 * @brief 遥控器数据解析
 *
 * @param sbus_buf 接收buffer
 */
static void sbus_to_fs(const uint8_t *sbus_buf)
{
#if defined(FS_SBUS) && !defined(FS_iBUS)
    if ((sbus_buf[0] == 0x0F)&&sbus_buf[24]==0x00) fs_rx_sta = 1;
    else fs_rx_sta=0;

    if(fs_rx_sta==1)
    {
        fs_ctrl[TEMP].rocker_r_= ((sbus_buf[1] | sbus_buf[2] << 8) & 0x07FF) - FS_DATA_MID;							//右拨杆水平
        fs_ctrl[TEMP].rocker_r1= ((sbus_buf[2] >> 3 | sbus_buf[3] << 5) & 0x07FF) - FS_DATA_MID;					//右拨杆垂直
        fs_ctrl[TEMP].rocker_l1= ((sbus_buf[3] >> 6 | sbus_buf[4] << 2 | sbus_buf[5] << 10) & 0x07FF) - FS_DATA_MID;	//左拨杆垂直
        fs_ctrl[TEMP].rocker_l_= ((sbus_buf[5] >> 1 | sbus_buf[6] << 7) & 0x07FF) - FS_DATA_MID;					//左拨杆水平

        fs_ctrl[TEMP].switch_l1= ((sbus_buf[6] >> 4 | sbus_buf[7] << 4) & 0x07FF);
        fs_ctrl[TEMP].switch_l2= ((sbus_buf[7] >> 7 | sbus_buf[8] << 1 | sbus_buf[9] << 9) & 0x07FF);
        fs_ctrl[TEMP].switch_r1= ((sbus_buf[9] >> 2 | sbus_buf[10] << 6) & 0x07FF);
        fs_ctrl[TEMP].switch_r2= ((sbus_buf[10] >> 5 | sbus_buf[11] << 3) & 0x07FF);

        fs_ctrl[TEMP].knob_l= ((sbus_buf[12] | sbus_buf[13] << 8) & 0x07FF) - FS_DATA_MIN;
        fs_ctrl[TEMP].knob_r = ((sbus_buf[13] >> 3 | sbus_buf[14] << 5) & 0x07FF) - FS_DATA_MIN;
        // fs_ctrl[TEMP] = ((sbus_buf[14] >> 6 | sbus_buf[15] << 2 | sbus_buf[16] << 10) & 0x07FF);
        // fs_ctrl[TEMP] = ((sbus_buf[16] >> 1 | sbus_buf[17] << 7) & 0x07FF);
        // fs_ctrl[TEMP] = ((sbus_buf[17] >> 4 | sbus_buf[18] << 4) & 0x07FF);
        // fs_ctrl[TEMP] = ((sbus_buf[18] >> 7 | sbus_buf[19] << 1 | sbus_buf[20] << 9) & 0x07FF);
        // fs_ctrl[TEMP] = ((sbus_buf[20] >> 2 | sbus_buf[21] << 6) & 0x07FF);
        // fs_ctrl[TEMP] = ((sbus_buf[21] >> 5 | sbus_buf[22] << 3) & 0x07FF);
        

        fs_data_dead_limit(fs_ctrl[TEMP].rocker_l_,5);
        fs_data_dead_limit(fs_ctrl[TEMP].rocker_l1,5);
        fs_data_dead_limit(fs_ctrl[TEMP].rocker_r_,5);
        fs_data_dead_limit(fs_ctrl[TEMP].rocker_r1,5);

        fs_data_change(fs_ctrl[TEMP].switch_l1);
        fs_data_change(fs_ctrl[TEMP].switch_l2);
        fs_data_change(fs_ctrl[TEMP].switch_r1);
        fs_data_change(fs_ctrl[TEMP].switch_r2);

        RectifyRCjoystick();
        fs_rx_sta = 0;                        // 准备下一次接收
    }
#endif

#if defined(FS_iBUS) && !defined(FS_SBUS)

    if (sbus_buf[0] == 0x20 && sbus_buf[1] == 0x40) fs_rx_sta = 1;
    else fs_rx_sta = 0;

    if(fs_rx_sta==1)
    {
        for(uint8_t i = 0; i < IBUS_USER_CHANNELS; i++)
        {
            if(i < 4)//摇杆
                *(&fs_ctrl[TEMP].rocker_r_ + i) = (uint16_t)(sbus_buf[i * 2 + 3] << 8 | sbus_buf[i * 2 + 2]) - FS_DATA_MID;
            else if(i == 8||i == 9)//旋钮
                *(&fs_ctrl[TEMP].rocker_r_ + i) = (uint16_t)(sbus_buf[i * 2 + 3] << 8 | sbus_buf[i * 2 + 2]) - FS_DATA_MIN;
            else
                *(&fs_ctrl[TEMP].rocker_r_ + i) = (uint16_t)(sbus_buf[i * 2 + 3] << 8 | sbus_buf[i * 2 + 2]);

        }

        fs_data_dead_limit(fs_ctrl[TEMP].rocker_l_,5);
        fs_data_dead_limit(fs_ctrl[TEMP].rocker_l1,5);
        fs_data_dead_limit(fs_ctrl[TEMP].rocker_r_,5);
        fs_data_dead_limit(fs_ctrl[TEMP].rocker_r1,5);

        fs_data_change(fs_ctrl[TEMP].switch_l1);
        fs_data_change(fs_ctrl[TEMP].switch_l2);
        fs_data_change(fs_ctrl[TEMP].switch_r1);
        fs_data_change(fs_ctrl[TEMP].switch_r2);

        RectifyRCjoystick();
        fs_rx_sta = 0;                        // 准备下一次接收
    }
#endif
    memcpy(&fs_ctrl[LAST], &fs_ctrl[TEMP], sizeof(FS_ctrl_t)); // 保存上一次的数据,用于按键持续按下和切换的判断
}

/**
 * @brief 对sbus_to_rc的简单封装,用于注册到bsp_usart的回调函数中
 *
 */
static void FSRxCallback()
{
    DaemonReload(fs_daemon_instance);         // 先喂狗
    sbus_to_fs(fs_usart_instance->recv_buff); // 进行协议解析
}

/**
 * @brief 遥控器离线的回调函数,注册到守护进程中,串口掉线时调用
 *
 */
static void FSLostCallback(void *id)
{
    memset(fs_ctrl, 0, sizeof(fs_ctrl)); // 清空遥控器数据
    USARTServiceInit(fs_usart_instance); // 尝试重新启动接收
    LOGWARNING("[rc] remote control lost");
}

FS_ctrl_t *FSControlInit(UART_HandleTypeDef *fs_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = FSRxCallback;
    conf.usart_handle = fs_usart_handle;
    conf.recv_buff_size = FS_FRAME_SIZE;
    fs_usart_instance = USARTRegister(&conf);

    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 10, // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
        .callback = FSLostCallback,
        .owner_id = NULL, // 只有1个遥控器,不需要owner_id
    };
    fs_daemon_instance = DaemonRegister(&daemon_conf);

    fs_init_flag = 1;
    return fs_ctrl;
}

uint8_t FSControlIsOnline()
{
    if (fs_init_flag)
        return DaemonIsOnline(fs_daemon_instance);
    return 0;
}