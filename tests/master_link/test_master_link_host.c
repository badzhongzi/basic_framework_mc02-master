/*
 * 主从链路协议的主机测试: 打包 -> 各种切分/干扰 -> 解析, 检查结果.
 * 在工程根目录运行 tests/master_link/run_host_test.sh
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "master_link.h"

static int failures;
#define CHECK(cond, msg)                                       \
    do                                                         \
    {                                                          \
        if (!(cond))                                           \
        {                                                      \
            printf("FAIL line %d: %s\n", __LINE__, msg);       \
            failures++;                                        \
        }                                                      \
    } while (0)

static void make_q(float q[6], int k)
{
    for (int j = 0; j < 6; j++)
        q[j] = (float)(0.1 * (j + 1) * (k % 7) - 1.0 + 0.001 * k);
}

static int frame_eq(const MasterLinkFrame_s *f, const float q[6], uint16_t flags)
{
    return memcmp(f->q, q, sizeof(f->q)) == 0 && f->flags == flags;
}

int main(void)
{
    uint8_t buf[MASTER_LINK_FRAME_LEN];
    float q[6];
    MasterLinkParser_s p;
    MasterLinkFrame_s out;

    /* 1. 长度和格式 */
    make_q(q, 3);
    uint16_t len = MasterLinkPack(q, 0x0123, buf);
    CHECK(len == 34 && len == MASTER_LINK_FRAME_LEN, "frame length 34");
    CHECK(buf[0] == 0xA5 && buf[1] == 26 && buf[2] == 0, "header");
    CHECK(buf[4] == 0x01 && buf[5] == 0x03, "cmd id little endian");
    CHECK(buf[6] == 0x23 && buf[7] == 0x01, "flags little endian");

    /* 2. 整帧一次喂入 */
    MasterLinkParserReset(&p);
    CHECK(MasterLinkParserFeed(&p, buf, len, &out) == 1, "whole frame decoded");
    CHECK(frame_eq(&out, q, 0x0123), "whole frame content");

    /* 3. 逐字节喂入 + 前面有垃圾 (含假帧头 0xA5) */
    MasterLinkParserReset(&p);
    uint8_t junk[] = {0x00, 0xA5, 0x11, 0xA5, 0xA5, 0x1A, 0x00, 0x55};
    int got = 0;
    for (size_t i = 0; i < sizeof(junk); i++)
        got += MasterLinkParserFeed(&p, &junk[i], 1, &out);
    for (int i = 0; i < len; i++)
        got += MasterLinkParserFeed(&p, &buf[i], 1, &out);
    CHECK(got == 1 && frame_eq(&out, q, 0x0123), "byte by byte with junk");

    /* 4. 多帧连在一起, 并且随机切块 */
    srand(1);
    MasterLinkParserReset(&p);
    uint8_t stream[MASTER_LINK_FRAME_LEN * 200];
    float q_last[6];
    size_t slen = 0;
    for (int k = 0; k < 200; k++)
    {
        make_q(q, k);
        slen += MasterLinkPack(q, (uint16_t)(k & 0xF) | MASTER_LINK_FLAG_VALID, stream + slen);
        memcpy(q_last, q, sizeof(q));
    }
    got = 0;
    for (size_t pos = 0; pos < slen;)
    {
        size_t chunk = 1 + (size_t)(rand() % 80);
        if (pos + chunk > slen)
            chunk = slen - pos;
        got += MasterLinkParserFeed(&p, stream + pos, (uint16_t)chunk, &out);
        pos += chunk;
    }
    CHECK(got == 200, "200 frames in random chunks");
    CHECK(frame_eq(&out, q_last, (199 & 0xF) | MASTER_LINK_FLAG_VALID), "last frame content");
    CHECK(p.frame_bad == 0 && p.bytes_dropped == 0, "no drops on clean stream");

    /* 5. 每一位翻转都必须被拒绝 (CRC 能查出所有单比特错误) */
    make_q(q, 5);
    MasterLinkPack(q, 0x0011, buf);
    int accepted = 0;
    for (int byte = 0; byte < len; byte++)
        for (int bit = 0; bit < 8; bit++)
        {
            uint8_t bad[MASTER_LINK_FRAME_LEN];
            memcpy(bad, buf, len);
            bad[byte] ^= (uint8_t)(1u << bit);
            MasterLinkParserReset(&p);
            accepted += MasterLinkParserFeed(&p, bad, len, &out);
        }
    CHECK(accepted == 0, "all single-bit errors rejected");

    /* 6. 坏帧后面紧跟好帧, 好帧要能解出来 */
    {
        uint8_t two[MASTER_LINK_FRAME_LEN * 2];
        MasterLinkPack(q, 0x0011, two);
        two[10] ^= 0x40;
        MasterLinkPack(q, 0x0012, two + len);
        MasterLinkParserReset(&p);
        got = MasterLinkParserFeed(&p, two, sizeof(two), &out);
        CHECK(got == 1 && frame_eq(&out, q, 0x0012), "good frame after corrupted one");
    }

    /* 7. NaN / 超范围的帧 CRC 正确也要拒绝 */
    {
        float qn[6] = {0, 0, NAN, 0, 0, 0};
        MasterLinkPack(qn, 0x0010, buf);
        MasterLinkParserReset(&p);
        CHECK(MasterLinkParserFeed(&p, buf, len, &out) == 0 && p.frame_bad == 1, "NaN rejected");
        float qb[6] = {0, 0, 0, 0, 0, 20.0f};
        MasterLinkPack(qb, 0x0010, buf);
        MasterLinkParserReset(&p);
        CHECK(MasterLinkParserFeed(&p, buf, len, &out) == 0, "out of range rejected");
    }

    /* 8. 长度字段被改成别的值(帧头 CRC8 重新算对), 不能越界读 */
    {
        extern uint8_t crc_8(const uint8_t *input_str, uint16_t num_bytes);
        uint8_t evil[MASTER_LINK_FRAME_LEN];
        MasterLinkPack(q, 0x0010, evil);
        evil[1] = 0xFF;
        evil[2] = 0xFF;
        evil[3] = crc_8(evil, 3);
        MasterLinkParserReset(&p);
        CHECK(MasterLinkParserFeed(&p, evil, len, &out) == 0, "bad length rejected");
    }

    if (failures == 0)
        printf("RESULT: PASS\n");
    else
        printf("RESULT: FAIL (%d)\n", failures);
    return failures ? 1 : 0;
}
