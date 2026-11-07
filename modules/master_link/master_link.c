/**
 * @file master_link.c
 * @brief 主从链路的串口收发. 主端只用 Tx 部分, 从端只用 Rx 部分. 帧格式见 master_link.h.
 */
#include "master_link.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "bsp_log.h"
#include <string.h>

/* ------------------------------ 主端: 发送 ------------------------------ */

static USARTInstance *tx_usart;
static uint8_t tx_buf[MASTER_LINK_FRAME_LEN]; // DMA 发送缓冲, 发送期间不能改
static uint8_t tx_heartbeat;
static uint32_t tx_busy_skip; // 上一帧还没发完而跳过的次数, 调试看

void MasterLinkTxInit(UART_HandleTypeDef *huart)
{
    USART_Init_Config_s conf = {
        .usart_handle = huart,
        .recv_buff_size = 8, // 不收数据, 但 bsp_usart 注册时会启动接收, 给一个小缓冲
        .module_callback = NULL,
    };
    tx_usart = USARTRegister(&conf);
}

void MasterLinkSend(const float q[MASTER_LINK_JOINT_NUM], uint8_t valid, uint8_t gravity, uint8_t buttons)
{
    if (tx_usart == NULL)
        return;
    // 不用 USARTIsReady(): 它写成了 gState | BUSY_TX, 永远返回 0
    if (tx_usart->usart_handle->gState != HAL_UART_STATE_READY)
    {
        tx_busy_skip++;
        return;
    }
    uint16_t flags = (uint16_t)(tx_heartbeat & MASTER_LINK_FLAG_HEARTBEAT_MASK);
    if (valid)
        flags |= MASTER_LINK_FLAG_VALID;
    if (gravity)
        flags |= MASTER_LINK_FLAG_GRAVITY_ON;
    flags |= (uint16_t)buttons << MASTER_LINK_FLAG_BUTTON_SHIFT;
    tx_heartbeat++;

    uint16_t len = MasterLinkPack(q, flags, tx_buf);
    USARTSend(tx_usart, tx_buf, len, USART_TRANSFER_DMA);
}

/* ------------------------------ 从端: 接收 ------------------------------ */

static USARTInstance *rx_usart;
static DaemonInstance *rx_daemon;
static MasterLinkParser_s rx_parser;
static MasterLinkFrame_s rx_frame;   // 最新一帧, 串口中断写, robottask 读
static uint32_t rx_frame_cnt;        // 收到的合法帧数
static uint8_t rx_last_heartbeat;
static uint8_t rx_heartbeat_stall;   // 心跳连续不变的帧数
static uint32_t rx_lost_cnt;

/* 串口中断里调用: 本次收到的字节数 = 缓冲区大小 - DMA 剩余 (idle 和 DMA 满两种情况 HAL 都会更新 RxXferCount) */
static void MasterLinkRxCallback()
{
    UART_HandleTypeDef *h = rx_usart->usart_handle;
    uint16_t n = (uint16_t)(h->RxXferSize - h->RxXferCount);
    MasterLinkFrame_s f;

    if (n > rx_usart->recv_buff_size)
        n = rx_usart->recv_buff_size;
    if (MasterLinkParserFeed(&rx_parser, rx_usart->recv_buff, n, &f) == 0)
        return;

    uint8_t hb = (uint8_t)(f.flags & MASTER_LINK_FLAG_HEARTBEAT_MASK);
    if (rx_frame_cnt > 0 && hb == rx_last_heartbeat)
    {
        if (rx_heartbeat_stall < 0xFF)
            rx_heartbeat_stall++;
    }
    else
        rx_heartbeat_stall = 0;
    rx_last_heartbeat = hb;
    rx_frame = f;
    rx_frame_cnt++;
    DaemonReload(rx_daemon); // 只有合法帧才喂狗
}

static void MasterLinkLostCallback(void *id)
{
    (void)id;
    MasterLinkParserReset(&rx_parser); // 丢掉半帧
    USARTServiceInit(rx_usart);        // 重新启动接收
    if (rx_lost_cnt++ % 100 == 0)      // daemon 离线后每 10 ms 调一次, 日志限频到 1 s
        LOGWARNING("[master_link] link lost");
}

void MasterLinkRxInit(UART_HandleTypeDef *huart)
{
    MasterLinkParserReset(&rx_parser);
    USART_Init_Config_s conf = {
        .usart_handle = huart,
        .recv_buff_size = 128, // 115200 下 10 ms 最多约 115 字节
        .module_callback = MasterLinkRxCallback,
    };
    rx_usart = USARTRegister(&conf);

    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 10, // 100 ms 没有合法帧算掉线
        .callback = MasterLinkLostCallback,
        .owner_id = NULL,
    };
    rx_daemon = DaemonRegister(&daemon_conf);
}

/* robottask 读中断写的数据: 短暂关中断拷一份, 避免读到两帧拼起来的数据 */
static void rx_snapshot(MasterLinkFrame_s *f, uint32_t *cnt, uint8_t *stall)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *f = rx_frame;
    *cnt = rx_frame_cnt;
    *stall = rx_heartbeat_stall;
    __set_PRIMASK(primask);
}

uint8_t MasterLinkIsValid(void)
{
    MasterLinkFrame_s f;
    uint32_t cnt;
    uint8_t stall;
    if (rx_usart == NULL)
        return 0;
    rx_snapshot(&f, &cnt, &stall);
    // daemon 注册时计数是满的, 上电 100 ms 内没收到帧也会报在线, 所以另外要求收到过帧
    return cnt > 0 && DaemonIsOnline(rx_daemon) &&
           stall < MASTER_LINK_HEARTBEAT_STALL_FRAMES &&
           (f.flags & MASTER_LINK_FLAG_VALID);
}

void MasterLinkGetQ(float q[MASTER_LINK_JOINT_NUM])
{
    MasterLinkFrame_s f;
    uint32_t cnt;
    uint8_t stall;
    rx_snapshot(&f, &cnt, &stall);
    memcpy(q, f.q, sizeof(f.q));
}

uint8_t MasterLinkGetButtons(void)
{
    MasterLinkFrame_s f;
    uint32_t cnt;
    uint8_t stall;
    rx_snapshot(&f, &cnt, &stall);
    return (uint8_t)((f.flags & MASTER_LINK_FLAG_BUTTON_MASK) >> MASTER_LINK_FLAG_BUTTON_SHIFT);
}
