/**
 * @file master_link_protocol.c
 * @brief 主从链路的打包和字节流解析, 不依赖 HAL. 帧格式见 master_link.h.
 *
 * 校验和打包复用 seasky_protocol. 但 get_protocol_info() 不检查长度字段, 坏帧头
 * (CRC8 只有 1 字节, 噪声下 1/256 的概率能过) 会让它按错误长度去读、去 memcpy,
 * 所以这里先自己核对帧头和长度, 确认整帧都在缓冲区里以后才交给它.
 */
#include "master_link.h"
#include "seasky_protocol.h"
#include "crc8.h"
#include <math.h>
#include <string.h>

uint16_t MasterLinkPack(const float q[MASTER_LINK_JOINT_NUM], uint16_t flags, uint8_t *buf)
{
    float tx[MASTER_LINK_JOINT_NUM]; // get_protocol_send_data 的参数不是 const
    uint16_t len = 0;
    memcpy(tx, q, sizeof(tx));
    get_protocol_send_data(MASTER_LINK_CMD_ID, flags, tx, MASTER_LINK_JOINT_NUM, buf, &len);
    return len;
}

void MasterLinkParserReset(MasterLinkParser_s *p)
{
    memset(p, 0, sizeof(*p));
}

static void parser_drop(MasterLinkParser_s *p, uint16_t n)
{
    if (n >= p->len)
    {
        p->len = 0;
        return;
    }
    memmove(p->buf, p->buf + n, p->len - n);
    p->len -= n;
}

/* 数值检查: 必须是有限值且在合理范围内 */
static uint8_t frame_values_ok(const MasterLinkFrame_s *f)
{
    for (uint8_t j = 0; j < MASTER_LINK_JOINT_NUM; j++)
        if (!isfinite(f->q[j]) || fabsf(f->q[j]) > MASTER_LINK_Q_ABS_MAX)
            return 0;
    return 1;
}

/* 从缓冲区开头反复解析, 直到数据不够一帧 */
static void parser_try(MasterLinkParser_s *p, MasterLinkFrame_s *out, uint16_t *decoded)
{
    while (p->len > 0)
    {
        // 1. 找帧头
        uint16_t sof = 0;
        while (sof < p->len && p->buf[sof] != PROTOCOL_CMD_ID)
            sof++;
        if (sof > 0)
        {
            p->bytes_dropped += sof;
            parser_drop(p, sof);
            continue;
        }
        // 2. 帧头 4 字节: CRC8 和长度字段
        if (p->len < 4)
            return;
        uint16_t data_len = (uint16_t)(p->buf[1] | (p->buf[2] << 8));
        if (crc_8(p->buf, 3) != p->buf[3] || data_len != MASTER_LINK_DATA_LEN)
        {
            p->bytes_dropped++; // 不是真帧头, 跳过这个 0xA5 重新找
            parser_drop(p, 1);
            continue;
        }
        // 3. 等整帧
        if (p->len < MASTER_LINK_FRAME_LEN)
            return;
        // 4. CRC16 + 命令号 + 数值
        MasterLinkFrame_s f;
        uint16_t flags = 0;
        uint8_t raw[4 * MASTER_LINK_JOINT_NUM];
        uint16_t cmd = get_protocol_info(p->buf, &flags, raw);
        if (cmd == MASTER_LINK_CMD_ID)
        {
            memcpy(f.q, raw, sizeof(f.q));
            f.flags = flags;
            if (frame_values_ok(&f))
            {
                *out = f;
                p->frame_ok++;
                (*decoded)++;
                parser_drop(p, MASTER_LINK_FRAME_LEN);
                continue;
            }
        }
        p->frame_bad++;
        parser_drop(p, 1); // 坏帧: 只跳 1 字节, 真帧头可能就在里面
    }
}

uint16_t MasterLinkParserFeed(MasterLinkParser_s *p, const uint8_t *data, uint16_t n, MasterLinkFrame_s *out)
{
    uint16_t decoded = 0;
    for (uint16_t i = 0; i < n; i++)
    {
        if (p->len >= sizeof(p->buf)) // 正常不会发生: 每进一个字节都会解析一次
        {
            p->bytes_dropped++;
            parser_drop(p, 1);
        }
        p->buf[p->len++] = data[i];
        if (p->len >= 4 || p->buf[0] != PROTOCOL_CMD_ID)
            parser_try(p, out, &decoded);
    }
    return decoded;
}
