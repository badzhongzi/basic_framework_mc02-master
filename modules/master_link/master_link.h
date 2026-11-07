/**
 * @file master_link.h
 * @brief 主从机械臂串口链路: 主臂(同构遥控器) -> 从臂, 单向.
 *
 * 本文件和 master_link.c / master_link_protocol.c 在主端、从端两个工程里是同一份,
 * 改协议时两边一起改.
 *
 * 帧格式沿用 seasky 协议 (modules/master_machine/seasky_protocol), 小端:
 *
 *   偏移  长度  内容
 *   0     1     SOF = 0xA5
 *   1     2     data_length = 26 (flags 2 字节 + 6 个 float)
 *   3     1     CRC8(字节 0..2)
 *   4     2     cmd_id = MASTER_LINK_CMD_ID
 *   6     2     flags, 见 MASTER_LINK_FLAG_*
 *   8     24    float q[6], 从臂 URDF 关节角, rad
 *   32    2     CRC16(字节 0..31)
 *   共 34 字节. 115200 波特率下约 3 ms, 主端 10 ms 发一帧.
 *
 * q 已经在主端换算成从臂的关节角 (同一个零点、同一个正方向), 从端直接当目标用.
 */
#ifndef MASTER_LINK_H
#define MASTER_LINK_H

#include <stdint.h>

#define MASTER_LINK_JOINT_NUM 6
#define MASTER_LINK_CMD_ID 0x0301
#define MASTER_LINK_DATA_LEN (2 + 4 * MASTER_LINK_JOINT_NUM) // seasky 的 data_length 字段
#define MASTER_LINK_FRAME_LEN (MASTER_LINK_DATA_LEN + 8)     // 整帧 34 字节

/* flags 位定义 */
#define MASTER_LINK_FLAG_HEARTBEAT_MASK 0x000Fu // bit0-3: 心跳计数, 每帧加 1
#define MASTER_LINK_FLAG_VALID 0x0010u          // bit4: 主端数据有效(6个电机都在线, 角度可信)
#define MASTER_LINK_FLAG_GRAVITY_ON 0x0020u     // bit5: 主端重力补偿正在出力(只用于显示)
#define MASTER_LINK_FLAG_BUTTON_SHIFT 8u        // bit8-15: 按钮位, bit8 预留给夹爪
#define MASTER_LINK_FLAG_BUTTON_MASK 0xFF00u

/* 心跳计数连续多少帧不变就认为主端卡住了 (10 帧 = 100 ms) */
#define MASTER_LINK_HEARTBEAT_STALL_FRAMES 10u
/* 角度合法范围, 超出按坏帧丢弃 (只挡明显错误的数据, 限位由从端 gimbal 夹) */
#define MASTER_LINK_Q_ABS_MAX 12.5f

/* 一帧解出来的内容 */
typedef struct
{
    float q[MASTER_LINK_JOINT_NUM]; // rad
    uint16_t flags;
} MasterLinkFrame_s;

/* ---------------- 纯计算部分 (master_link_protocol.c, 不依赖 HAL, 可在 PC 上测) ---------------- */

/**
 * @brief 打包一帧. buf 至少 MASTER_LINK_FRAME_LEN 字节.
 * @return 帧长度
 */
uint16_t MasterLinkPack(const float q[MASTER_LINK_JOINT_NUM], uint16_t flags, uint8_t *buf);

/* 字节流解析器: 能处理一次收到半帧、多帧、帧前有垃圾字节的情况 */
typedef struct
{
    uint8_t buf[MASTER_LINK_FRAME_LEN * 2];
    uint16_t len;
    uint32_t frame_ok;      // 解出的合法帧数
    uint32_t frame_bad;     // CRC/长度/命令号/数值错误的帧数
    uint32_t bytes_dropped; // 为了重新找帧头丢掉的字节数
} MasterLinkParser_s;

void MasterLinkParserReset(MasterLinkParser_s *p);

/**
 * @brief 喂入收到的字节, 解出合法帧时写入 out (多帧时是最新一帧).
 * @return 本次解出的合法帧数
 */
uint16_t MasterLinkParserFeed(MasterLinkParser_s *p, const uint8_t *data, uint16_t n, MasterLinkFrame_s *out);

/* ---------------- 串口收发部分 (master_link.c, 依赖 bsp_usart / daemon) ---------------- */
#ifndef MASTER_LINK_HOST_TEST
#include "usart.h"

/* 主端: 注册发送串口 */
void MasterLinkTxInit(UART_HandleTypeDef *huart);
/**
 * @brief 主端: 发一帧. 心跳计数在内部维护.
 * @param valid   主端数据是否可信
 * @param gravity 重力补偿是否正在出力
 * @param buttons 按钮位 (bit0 对应 flags bit8)
 */
void MasterLinkSend(const float q[MASTER_LINK_JOINT_NUM], uint8_t valid, uint8_t gravity, uint8_t buttons);

/* 从端: 注册接收串口和 daemon (100 ms 收不到合法帧就算掉线) */
void MasterLinkRxInit(UART_HandleTypeDef *huart);
/**
 * @brief 从端: 链路是否可用 = 在线 && 心跳在变 && 主端报告数据有效.
 *        不可用时不要使用 MasterLinkGetQ() 的结果.
 */
uint8_t MasterLinkIsValid(void);
/* 从端: 取最新一帧的关节角 (rad, 从臂 URDF 关节角) */
void MasterLinkGetQ(float q[MASTER_LINK_JOINT_NUM]);
/* 从端: 最新一帧的按钮位 */
uint8_t MasterLinkGetButtons(void);
#endif // MASTER_LINK_HOST_TEST

#endif // MASTER_LINK_H
