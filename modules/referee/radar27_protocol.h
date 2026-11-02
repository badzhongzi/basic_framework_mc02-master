/**
 * @file radar27_protocol.h
 * @brief RoboMaster 2027 通信协议 —— 雷达无线链路数据定义 (0x0A01 ~ 0x0A06)
 *
 * @note 依据手册 1.5 节 雷达无线链路数据说明。
 *
 * @note 链路性质: "信号发射源 -> 雷达"的**单向**广播, 每个命令码都以 10Hz 定频发送,
 *       合计 24+12+10+12+47+6 = 111 字节 / 100ms (约 1.1KB/s)。
 *       由于它是无线广播而不是串口点对点数据, 实际工程中通常由官方雷达接收机
 *       解析后通过串口/网口转发给主控; 若接收机以相同帧格式从串口输出,
 *       可直接复用 rm_referee27 中的解析器(R27_LINK_RADAR)。
 *       若接收机走以太网, 则不应塞进本模块, 需另建独立的网络解析模块。
 *
 * @note 所有位定义都是"对方视角": bit=1 表示对方激活/对方被检测到。
 */

#ifndef RADAR27_PROTOCOL_H
#define RADAR27_PROTOCOL_H

#include "stdint.h"
#include "referee27_protocol.h" /* 复用 r27_buff_t / r27_frame_header_t */

/* ======================== 命令码 ======================== */

typedef enum
{
    R27_ID_radar_robot_pos = 0x0A01,   /* 对方机器人位置坐标, 10Hz */
    R27_ID_radar_robot_hp = 0x0A02,    /* 对方机器人血量信息, 10Hz */
    R27_ID_radar_robot_ammo = 0x0A03,  /* 对方机器人剩余发弹量, 10Hz */
    R27_ID_radar_team_status = 0x0A04, /* 对方队伍宏观状态, 10Hz */
    R27_ID_radar_robot_buff = 0x0A05,  /* 对方各机器人当前增益效果, 10Hz */
    R27_ID_radar_key = 0x0A06,         /* 对方干扰波密钥, 10Hz */
} R27RadarCmdID_e;

typedef enum
{
    R27_LEN_radar_robot_pos = 24,   /* 0x0A01 */
    R27_LEN_radar_robot_hp = 12,    /* 0x0A02 */
    R27_LEN_radar_robot_ammo = 10,  /* 0x0A03 */
    R27_LEN_radar_team_status = 12, /* 0x0A04 */
    R27_LEN_radar_robot_buff = 47,  /* 0x0A05 */
    R27_LEN_radar_key = 6,          /* 0x0A06 */
} R27RadarDataLength_e;

#pragma pack(1)

/* ======================== 数据段结构体 ======================== */

/* ID: 0x0A01  Byte: 24  对方机器人的位置坐标 (表 1-41), 单位 cm
 * 结构/顺序与 0x0305(选手端小地图) 中"对方机器人"部分一致 */
typedef struct
{
    uint16_t opponent_heavyengineer_position_x;
    uint16_t opponent_heavyengineer_position_y;
    uint16_t reserved_1;
    uint16_t reserved_2;
    uint16_t opponent_infantry_3_position_x;
    uint16_t opponent_infantry_3_position_y;
    uint16_t opponent_infantry_4_position_x;
    uint16_t opponent_infantry_4_position_y;
    uint16_t reserved_3;
    uint16_t reserved_4;
    uint16_t opponent_sentry_position_x;
    uint16_t opponent_sentry_position_y;
} r27_radar_robot_pos_t;

/* ID: 0x0A02  Byte: 12  对方机器人的血量信息 (表 1-42)
 * 未上场或被罚下的机器人在此处的血量为 0 */
typedef struct
{
    uint16_t opponent_1_robot_HP; /* 对方 1 号重装血量 */
    uint16_t reserved;
    uint16_t opponent_3_robot_HP; /* 对方 3 号步兵血量 */
    uint16_t opponent_4_robot_HP; /* 对方 4 号步兵血量 */
    uint16_t reserved_2;
    uint16_t opponent_7_robot_HP; /* 对方 7 号哨兵血量 */
} r27_radar_robot_hp_t;

/* ID: 0x0A03  Byte: 10  对方机器人的剩余发弹量信息 (表 1-43)
 * 3/4 号步兵的允许发弹量已含堡垒提供的储备允许发弹量 */
typedef struct
{
    uint16_t opponent_1_robot_ammo; /* 对方 1 号重装允许发弹量 */
    uint16_t opponent_3_robot_ammo; /* 对方 3 号步兵允许发弹量 */
    uint16_t opponent_4_robot_ammo; /* 对方 4 号步兵允许发弹量 */
    uint16_t reserved;
    uint16_t opponent_7_robot_ammo; /* 对方 7 号哨兵允许发弹量 */
} r27_radar_robot_ammo_t;

/* ID: 0x0A04  Byte: 12  对方队伍的宏观状态信息 (表 1-44) */
typedef struct
{
    uint16_t remaining_gold_coin; /* 对方剩余金币数 */
    uint16_t total_gold_coin;     /* 对方累计总金币数 */
    uint32_t event_data;          /* 对方视角的增益点/控制区状态, 位定义见下 */
    uint32_t event_data_2;        /* 对方视角的地形跨越增益点与阵地控制状态, 位定义见下 */
} r27_radar_team_status_t;

/* 0x0A04 event_data (offset 4) 位定义 —— 注意是"对方视角" */
#define R27_RADAR_EVENT_SUPPLY_AREA_SHIFT 0 /* bit0    对方补给区激活: 0未激活 1对方激活 */
#define R27_RADAR_EVENT_SUPPLY_AREA_MASK 0x1u
#define R27_RADAR_EVENT_FORT_R_SHIFT 1 /* bit1-3  R 堡垒: 0未激活 1对方 2己方 3双方, 未控制阵地时为4 */
#define R27_RADAR_EVENT_FORT_R_MASK 0x7u
#define R27_RADAR_EVENT_FORT_B_SHIFT 4 /* bit4-6  B 堡垒 */
#define R27_RADAR_EVENT_FORT_B_MASK 0x7u
#define R27_RADAR_EVENT_BASE_R_SHIFT 7 /* bit7-9  R 基地增益点 */
#define R27_RADAR_EVENT_BASE_R_MASK 0x7u
#define R27_RADAR_EVENT_BASE_B_SHIFT 10 /* bit10-12 B 基地增益点 */
#define R27_RADAR_EVENT_BASE_B_MASK 0x7u
#define R27_RADAR_EVENT_HIGHLAND_SHIFT 13 /* bit13   对方制高点 */
#define R27_RADAR_EVENT_HIGHLAND_MASK 0x1u
#define R27_RADAR_EVENT_CENTER_R_SHIFT 14 /* bit14-15 R 中央高地: 0未激活 1对方 2己方, 未控制阵地时为3 */
#define R27_RADAR_EVENT_CENTER_R_MASK 0x3u
#define R27_RADAR_EVENT_CENTER_B_SHIFT 16 /* bit16-17 B 中央高地 */
#define R27_RADAR_EVENT_CENTER_B_MASK 0x3u
#define R27_RADAR_EVENT_OUTPOST_R_SHIFT 18 /* bit18-20 R 前哨增益点 */
#define R27_RADAR_EVENT_OUTPOST_R_MASK 0x7u
#define R27_RADAR_EVENT_OUTPOST_B_SHIFT 21 /* bit21-23 B 前哨增益点 */
#define R27_RADAR_EVENT_OUTPOST_B_MASK 0x7u

/* 0x0A04 event_data_2 (offset 8) 位定义 */
#define R27_RADAR_EVENT2_CROSS_CENTER_HIGHLAND_OPP_SHIFT 0  /* bit0 对方地形跨越(中央高地上方) */
#define R27_RADAR_EVENT2_CROSS_CENTER_HIGHLAND_ALLY_SHIFT 1 /* bit1 己方地形跨越(中央高地上方) */
#define R27_RADAR_EVENT2_CROSS_ROAD_OPP_SHIFT 2             /* bit2 对方地形跨越(公路上方) */
#define R27_RADAR_EVENT2_CROSS_ROAD_ALLY_SHIFT 3            /* bit3 己方地形跨越(公路上方) */
#define R27_RADAR_EVENT2_TUNNEL_OPP_ROAD_DOWN_SHIFT 4       /* bit4-6  对方隧道(靠对方公路一侧 下/中/上) */
#define R27_RADAR_EVENT2_TUNNEL_OPP_NEAR_LOW_SHIFT 7        /* bit7-9  隧道(靠中央高地, 靠对方梯形高地) */
#define R27_RADAR_EVENT2_TUNNEL_OPP_FAR_SHIFT 10            /* bit10-12 隧道(远离中央高地, 靠对方梯形) */
#define R27_RADAR_EVENT2_TUNNEL_ALLY_ROAD_DOWN_SHIFT 13     /* bit13-15 己方隧道(靠己方公路一侧 下/中/上) */
#define R27_RADAR_EVENT2_TUNNEL_ALLY_NEAR_LOW_SHIFT 16      /* bit16-18 隧道(靠中央高地, 靠己方梯形高地) */
#define R27_RADAR_EVENT2_TUNNEL_ALLY_FAR_SHIFT 19           /* bit19-21 隧道(远离中央高地, 靠己方梯形) */
#define R27_RADAR_EVENT2_OUTPOST_SITE_R_SHIFT 22            /* bit22-23 R 前哨阵地: 0对方完全 1对方临时 2对方未控制 */
#define R27_RADAR_EVENT2_OUTPOST_SITE_R_MASK 0x3u
#define R27_RADAR_EVENT2_OUTPOST_SITE_B_SHIFT 24 /* bit24-25 B 前哨阵地 */
#define R27_RADAR_EVENT2_OUTPOST_SITE_B_MASK 0x3u
#define R27_RADAR_EVENT2_BASE_SITE_R_SHIFT 26 /* bit26-27 R 基地阵地 */
#define R27_RADAR_EVENT2_BASE_SITE_R_MASK 0x3u
#define R27_RADAR_EVENT2_BASE_SITE_B_SHIFT 28 /* bit28-29 B 基地阵地 */
#define R27_RADAR_EVENT2_BASE_SITE_B_MASK 0x3u

/* 机器人主要状态(0x0A05 offset 36~40) */
typedef enum
{
    R27_RADAR_ROBOT_ALIVE = 0,      /* 存活 */
    R27_RADAR_ROBOT_DEAD = 1,       /* 战亡 */
    R27_RADAR_ROBOT_INVINCIBLE = 2, /* 无敌但不虚弱 */
    R27_RADAR_ROBOT_WEAKENED = 3,   /* 无敌且虚弱 */
} R27RadarRobotState_e;

/* 机器人姿态(0x0A05 offset 35) */
typedef enum
{
    R27_RADAR_POSTURE_ATTACK = 1,
    R27_RADAR_POSTURE_DEFENCE = 2,
    R27_RADAR_POSTURE_MOVE = 3,
    R27_RADAR_POSTURE_ATTACK_ENHANCED = 4,
    R27_RADAR_POSTURE_DEFENCE_ENHANCED = 5,
    R27_RADAR_POSTURE_MOVE_ENHANCED = 6,
} R27RadarPosture_e;

/* ID: 0x0A05  Byte: 47  对方各机器人当前增益效果 (表 1-45)
 * 每台机器人的增益结构完全相同(复用 r27_buff_t, 7 字节) */
typedef struct
{
    r27_buff_t heavyengineer_buff;   /* offset 0,  对方重装增益 */
    uint8_t reserved_1[7];           /* offset 7,  保留位 */
    r27_buff_t standard_3_buff;      /* offset 14, 对方 3 号步兵增益 */
    r27_buff_t standard_4_buff;      /* offset 21, 对方 4 号步兵增益 */
    r27_buff_t sentry_buff;          /* offset 28, 对方哨兵增益 */
    uint8_t sentry_posture;          /* offset 35, 对方哨兵当前姿态 (R27RadarPosture_e) */
    uint8_t heavyengineer_state;     /* offset 36, 对方重装主要状态 (R27RadarRobotState_e) */
    uint8_t reserved_2;              /* offset 37, 保留位 */
    uint8_t standard_3_state;        /* offset 38, 对方 3 号步兵主要状态 */
    uint8_t standard_4_state;        /* offset 39, 对方 4 号步兵主要状态 */
    uint8_t sentry_state;            /* offset 40, 对方哨兵主要状态 */
    uint16_t fortify_center_x;       /* offset 41, 对方自定义装甲加固区圆心 x, mm */
    uint16_t fortify_center_y;       /* offset 43, 对方自定义装甲加固区圆心 y, mm */
    uint16_t fortify_radius;         /* offset 45, 加固区半径 mm, 0 表示暂无加固情报 */
} r27_radar_robot_buff_t;

/* ID: 0x0A06  Byte: 6  对方干扰波密钥 (表 1-46)
 * 6 个字节均为 ASCII 编码的字母或数字, 用于 0x0121 中"验证破解的对方密钥" */
typedef struct
{
    uint8_t key[6];
} r27_radar_key_t;

/* ======================== 编译期长度校验 ======================== */

_Static_assert(sizeof(r27_radar_robot_pos_t) == R27_LEN_radar_robot_pos, "0x0A01 length mismatch");
_Static_assert(sizeof(r27_radar_robot_hp_t) == R27_LEN_radar_robot_hp, "0x0A02 length mismatch");
_Static_assert(sizeof(r27_radar_robot_ammo_t) == R27_LEN_radar_robot_ammo, "0x0A03 length mismatch");
_Static_assert(sizeof(r27_radar_team_status_t) == R27_LEN_radar_team_status, "0x0A04 length mismatch");
_Static_assert(sizeof(r27_radar_robot_buff_t) == R27_LEN_radar_robot_buff, "0x0A05 length mismatch");
_Static_assert(sizeof(r27_radar_key_t) == R27_LEN_radar_key, "0x0A06 length mismatch");

_Static_assert(offsetof(r27_radar_robot_buff_t, sentry_posture) == 35, "0x0A05 sentry_posture offset");
_Static_assert(offsetof(r27_radar_robot_buff_t, sentry_state) == 40, "0x0A05 sentry_state offset");
_Static_assert(offsetof(r27_radar_robot_buff_t, fortify_center_x) == 41, "0x0A05 fortify_center_x offset");
_Static_assert(offsetof(r27_radar_team_status_t, event_data) == 4, "0x0A04 event_data offset");
_Static_assert(offsetof(r27_radar_team_status_t, event_data_2) == 8, "0x0A04 event_data_2 offset");

#pragma pack()

#endif /* RADAR27_PROTOCOL_H */
