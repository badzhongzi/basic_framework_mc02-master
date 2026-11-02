/**
 * @file referee27_protocol.h
 * @brief RoboMaster 2027 机甲大师高校系列赛通信协议 V1.0.0 (20261029) 数据定义
 *
 * @note 依据手册:
 *       - 1.1 串口协议格式
 *       - 1.2 命令码 ID 和常规链路数据说明 (0x0001 ~ 0x0308)
 *       - 1.3 小地图交互数据 (0x0303 / 0x0305 / 0x0307 / 0x0308)
 *       - 1.4 图传链路数据说明 (0x0310 / 0x0311)
 *       - 附录一 CRC 校验代码 (与 modules/referee/crc_ref.c 完全一致, 直接复用)
 *       - 附录二 ID 编号说明
 *       雷达无线链路 (0x0A01 ~ 0x0A06) 见 radar27_protocol.h
 *
 * @note 本文件与 modules/referee/referee_protocol.h(2022/2023 协议) 并存,
 *       因此所有标识符统一加 R27_ 前缀, 结构体统一加 r27_ 前缀, 避免符号冲突。
 *
 * @note 所有结构体均按手册的字节偏移逐字段对应, 并配有 _Static_assert 做长度校验,
 *       若手册更新导致长度不符, 编译期即会报错。
 *
 * @warning 帧头/命令码均为小端(little-endian)存放, 位域按 LSB-first 排列,
 *          与本工程目标平台(Cortex-M4, little-endian)一致。
 */

#ifndef REFEREE27_PROTOCOL_H
#define REFEREE27_PROTOCOL_H

#include "stdint.h"
#include "stddef.h"

/* ============================== 版本与常量 ============================== */

#define R27_PROTOCOL_VERSION "RoboMaster 2027 通信协议 V1.0.0 (20261029)"

#define R27_REFEREE_SOF 0xA5u /* 帧起始字节, 固定 0xA5 */

/* 机器人颜色 */
#define R27_ROBOT_RED 0
#define R27_ROBOT_BLUE 1
/* 机器人 ID < 100 为红方, >= 100 为蓝方 */
#define R27_ROBOT_ID_COLOR_SPLIT 100u

/* 选手端(客户端) ID = 0x0100 + 机器人 ID, 例: 红 1 -> 0x0101, 蓝 107? (哨兵无选手端) */
#define R27_CLIENT_ID_OF_ROBOT(robot_id) ((uint16_t)(0x0100u + (uint16_t)(robot_id)))
/* 裁判系统服务器 ID, 用于哨兵/雷达自主决策指令(0x0120 / 0x0121)的接收者 */
#define R27_SERVER_CLIENT_ID 0x8080u

/* 自定义机器人间交互数据长度(子内容 ID 0x0200~0x02FF), 可按需修改 */
#define R27_COMMUNICATE_DATA_LEN 5
/* 0x0301 数据段: 子内容头 6 字节 + 内容数据段(最大 112 字节) */
#define R27_INTERACTION_HEAD_LEN 6
#define R27_INTERACTION_DATA_MAX 112

#pragma pack(1)

/* ============================== 帧格式 ============================== */

/* 通信协议偏移 */
typedef enum
{
    R27_FRAME_HEADER_OFFSET = 0, /* 帧头偏移 */
    R27_CMD_ID_OFFSET = 5,       /* 命令码偏移 */
    R27_DATA_OFFSET = 7,         /* 数据段偏移 */
} R27JudgeFrameOffset_e;

/* 通信协议长度 */
typedef enum
{
    R27_LEN_HEADER = 5, /* 帧头(SOF + data_length + seq + CRC8) */
    R27_LEN_CMDID = 2,  /* 命令码 */
    R27_LEN_TAIL = 2,   /* 帧尾 CRC16 */
    R27_LEN_CRC8 = 4,   /* 参与 CRC8 校验的长度: 帧头前 4 字节 */
} R27JudgeFrameLength_e;

/* 帧头偏移 */
typedef enum
{
    R27_SOF = 0,         /* 起始位 */
    R27_DATA_LENGTH = 1, /* 帧内数据长度(2 字节, 小端) */
    R27_SEQ = 3,         /* 包序号 */
    R27_CRC8 = 4,        /* CRC8 */
} R27FrameHeaderOffset_e;

/* 帧头定义 (5 字节) */
typedef struct
{
    uint8_t SOF;
    uint16_t DataLength;
    uint8_t Seq;
    uint8_t CRC8;
} r27_frame_header_t;

/* ============================== 命令码 ID ============================== */

typedef enum
{
    /* ---- 常规链路: 服务器 -> 机器人 ---- */
    R27_ID_game_state = 0x0001,           /* 比赛状态数据, 1Hz */
    R27_ID_game_result = 0x0002,          /* 比赛结果数据, 比赛结束触发 */
    R27_ID_game_robot_HP = 0x0003,        /* 机器人血量数据, 3Hz */
    R27_ID_event_data = 0x0101,           /* 场地事件数据, 1Hz */
    R27_ID_referee_warning = 0x0104,      /* 裁判警告数据, 触发 + 1Hz */
    R27_ID_dart_info = 0x0105,            /* 飞镖发射相关数据, 1Hz */
    /* ---- 常规链路: 主控模块 <-> 机器人 ---- */
    R27_ID_robot_status = 0x0201,         /* 机器人性能体系数据, 10Hz */
    R27_ID_power_heat_data = 0x0202,      /* 实时底盘缓冲能量和射击热量, 10Hz */
    R27_ID_robot_pos = 0x0203,            /* 机器人位置数据, 1Hz */
    R27_ID_robot_buff = 0x0204,           /* 机器人增益数据, 3Hz */
    R27_ID_hurt_data = 0x0206,            /* 伤害状态数据, 受伤触发 */
    R27_ID_shoot_data = 0x0207,           /* 实时射击数据, 发弹触发 */
    /* ---- 常规链路: 服务器 -> 特定机器人 ---- */
    R27_ID_projectile_allowance = 0x0208, /* 允许发弹量与剩余金币, 10Hz(重装/步兵/哨兵) */
    R27_ID_rfid_status = 0x0209,          /* RFID 模块状态, 3Hz */
    R27_ID_dart_client_cmd = 0x020A,      /* 飞镖选手端指令数据, 3Hz */
    R27_ID_ground_robot_position = 0x020B,/* 地面机器人位置数据, 1Hz(哨兵) */
    R27_ID_radar_mark_data = 0x020C,      /* 雷达标记进度数据, 1Hz(雷达) */
    R27_ID_sentry_info = 0x020D,          /* 哨兵自主决策信息同步, 1Hz(哨兵) */
    R27_ID_radar_info = 0x020E,           /* 雷达自主决策信息同步, 1Hz(雷达) */
    /* ---- 常规链路: 交互 ---- */
    R27_ID_robot_interaction = 0x0301,    /* 机器人交互数据, 上限 30Hz */
    /* ---- 常规链路: 小地图 ---- */
    R27_ID_map_command = 0x0303,          /* 选手端小地图交互数据(下发) */
    R27_ID_map_robot_data = 0x0305,       /* 选手端小地图接收雷达数据, 上限 5Hz */
    R27_ID_map_data = 0x0307,             /* 选手端小地图接收路径数据, 上限 1Hz */
    R27_ID_custom_info = 0x0308,          /* 选手端小地图接收机器人数据, 上限 3Hz */
    /* ---- 图传链路 ---- */
    R27_ID_robot_custom_data = 0x0310,    /* 机器人 -> 自定义终端, 上限 50Hz */
    R27_ID_custom_terminal_cmd = 0x0311,  /* 自定义终端 -> 机器人, 上限 75Hz */
} R27CmdID_e;

/* 命令码数据段长度(与手册表 1-5 一一对应) */
typedef enum
{
    R27_LEN_game_state = 11,              /* 0x0001 */
    R27_LEN_game_result = 1,              /* 0x0002 */
    R27_LEN_game_robot_HP = 24,           /* 0x0003   (旧协议为 32) */
    R27_LEN_event_data = 8,               /* 0x0101   (旧协议为 4) */
    R27_LEN_referee_warning = 3,          /* 0x0104 新增 */
    R27_LEN_dart_info = 3,                /* 0x0105 新增 */
    R27_LEN_robot_status = 17,            /* 0x0201   (旧协议为 13) */
    R27_LEN_power_heat_data = 14,         /* 0x0202   (旧协议为 16) */
    R27_LEN_robot_pos = 12,               /* 0x0203   (旧协议为 16) */
    R27_LEN_robot_buff = 7,               /* 0x0204   (旧协议为 6) */
    R27_LEN_hurt_data = 7,                /* 0x0206   (旧协议为 1) */
    R27_LEN_shoot_data = 7,               /* 0x0207 */
    R27_LEN_projectile_allowance = 8,     /* 0x0208 新增 */
    R27_LEN_rfid_status = 6,              /* 0x0209 新增 */
    R27_LEN_dart_client_cmd = 6,          /* 0x020A 新增 */
    R27_LEN_ground_robot_position = 40,   /* 0x020B 新增 */
    R27_LEN_radar_mark_data = 2,          /* 0x020C 新增 */
    R27_LEN_sentry_info = 14,             /* 0x020D 新增 */
    R27_LEN_radar_info = 8,               /* 0x020E 新增 */
    R27_LEN_robot_interaction = 118,      /* 0x0301   (6 + 112) */
    R27_LEN_map_command = 12,             /* 0x0303 新增 */
    R27_LEN_map_robot_data = 48,          /* 0x0305 新增 */
    R27_LEN_map_data = 105,               /* 0x0307 新增 */
    R27_LEN_custom_info = 34,             /* 0x0308 新增 */
    R27_LEN_robot_custom_data = 300,      /* 0x0310 新增(图传链路) */
    R27_LEN_custom_terminal_cmd = 60,     /* 0x0311 新增(图传链路) */
} R27JudgeDataLength_e;

/* ======================== 常规链路: 服务器 -> 机器人 ======================== */

/* ID: 0x0001  Byte: 11  比赛状态数据 (表 1-6) */
typedef struct
{
    uint8_t game_type : 4;     /* bit0-3: 1 RMUC / 2 RMUL单项赛 / 3 ICRA / 4 RMUL 3v3 / 5 RMUL 步兵对抗 */
    uint8_t game_progress : 4; /* bit4-7: 0未开始 1准备 2自检 3五秒倒计时 4比赛中 5结算中 */
    uint16_t stage_remain_time;/* 当前阶段剩余时间, 单位 s */
    uint64_t SyncTimeStamp;    /* UNIX 时间戳, 接 NTP 后生效 */
} r27_game_status_t;

/* ID: 0x0002  Byte: 1  比赛结果数据 (表 1-7) */
typedef struct
{
    uint8_t winner; /* 0 平局 / 1 红方胜 / 2 蓝方胜 */
} r27_game_result_t;

/* ID: 0x0003  Byte: 24  机器人血量数据 (表 1-8)
 * 注意: 新协议只给出"己方"数据 + 伤害差 + 双方控制度, 不再有 16 个血量字段 */
typedef struct
{
    uint16_t ally_1_robot_HP;     /* 己方 1 号重装血量 */
    uint16_t reserved;            /* 保留位 */
    uint16_t ally_3_robot_HP;     /* 己方 3 号步兵血量 */
    uint16_t ally_4_robot_HP;     /* 己方 4 号步兵血量 */
    int16_t damage_difference;    /* 己方全队总伤害与对方全队总伤害之差(有符号) */
    uint16_t ally_7_robot_HP;     /* 己方 7 号哨兵血量 */
    uint16_t outpost_R_HP;        /* R 前哨血量 */
    uint16_t ally_base_HP;        /* 己方基地血量 */
    uint16_t outpost_B_HP;        /* B 前哨血量 */
    uint16_t enemy_base_HP;       /* 对方基地血量 */
    uint16_t ally_control_degree; /* 己方累计控制度 */
    uint16_t enemy_control_degree;/* 对方累计控制度 */
} r27_game_robot_HP_t;

/* ID: 0x0101  Byte: 8  场地事件数据 (表 1-9)
 * event_data   : 己方视角的增益点/控制区状态
 * event_data_2 : 基地阵地控制状态 / 对方飞镖命中信息 / RMUL 控制区状态
 */
typedef struct
{
    uint32_t event_data;
    uint32_t event_data_2;
} r27_event_data_t;

/* event_data (0x0101 offset 0, 4 字节) 位定义 */
#define R27_EVENT_SUPPLY_AREA_SHIFT 0 /* bit0    己方补给区激活: 0未激活 1己方激活 */
#define R27_EVENT_SUPPLY_AREA_MASK 0x1u
#define R27_EVENT_SMALL_RUNE_SHIFT 1 /* bit1-2  己方小能量机关: 0未激活 1已激活 2正在激活 */
#define R27_EVENT_SMALL_RUNE_MASK 0x3u
#define R27_EVENT_BIG_RUNE_SHIFT 3 /* bit3-4  己方大能量机关: 同上 */
#define R27_EVENT_BIG_RUNE_MASK 0x3u
#define R27_EVENT_FORT_R_SHIFT 5 /* bit5-7  R 堡垒增益点: 0未激活 1己方 2对方 3双方, 未控制阵地时为4 */
#define R27_EVENT_FORT_R_MASK 0x7u
#define R27_EVENT_FORT_B_SHIFT 8 /* bit8-10 B 堡垒增益点 */
#define R27_EVENT_FORT_B_MASK 0x7u
#define R27_EVENT_BASE_R_SHIFT 11 /* bit11-13 R 基地增益点 */
#define R27_EVENT_BASE_R_MASK 0x7u
#define R27_EVENT_BASE_B_SHIFT 14 /* bit14-16 B 基地增益点 */
#define R27_EVENT_BASE_B_MASK 0x7u
#define R27_EVENT_HIGHLAND_SHIFT 17 /* bit17   己方制高点: 0未激活 1己方激活 */
#define R27_EVENT_HIGHLAND_MASK 0x1u
#define R27_EVENT_CENTER_R_SHIFT 18 /* bit18-19 R 中央高地: 0未激活 1己方 2对方, 未控制阵地时为3 */
#define R27_EVENT_CENTER_R_MASK 0x3u
#define R27_EVENT_CENTER_B_SHIFT 20 /* bit20-21 B 中央高地 */
#define R27_EVENT_CENTER_B_MASK 0x3u
#define R27_EVENT_OUTPOST_R_SHIFT 22 /* bit22-24 R 前哨增益点 */
#define R27_EVENT_OUTPOST_R_MASK 0x7u
#define R27_EVENT_OUTPOST_B_SHIFT 25 /* bit25-27 B 前哨增益点 */
#define R27_EVENT_OUTPOST_B_MASK 0x7u
#define R27_EVENT_OUTPOST_SITE_R_SHIFT 28 /* bit28-29 R 前哨阵地: 0完全 1临时 2未控制 */
#define R27_EVENT_OUTPOST_SITE_R_MASK 0x3u
#define R27_EVENT_OUTPOST_SITE_B_SHIFT 30 /* bit30-31 B 前哨阵地 */
#define R27_EVENT_OUTPOST_SITE_B_MASK 0x3u

/* event_data_2 (0x0101 offset 4, 4 字节) 位定义 */
#define R27_EVENT2_BASE_SITE_R_SHIFT 0 /* bit0-1  R 基地阵地: 0完全 1临时 2未控制 */
#define R27_EVENT2_BASE_SITE_R_MASK 0x3u
#define R27_EVENT2_BASE_SITE_B_SHIFT 2 /* bit2-3  B 基地阵地 */
#define R27_EVENT2_BASE_SITE_B_MASK 0x3u
#define R27_EVENT2_DART_HIT_TIME_SHIFT 4 /* bit4-12 对方飞镖最后击中前哨/基地的时间, s, 0~420 */
#define R27_EVENT2_DART_HIT_TIME_MASK 0x1FFu
#define R27_EVENT2_DART_HIT_TARGET_SHIFT 13 /* bit13-15 0默认 1R前哨 2基地固定 3基地随机固定 4基地随机移动 5基地末端移动 6B前哨 */
#define R27_EVENT2_DART_HIT_TARGET_MASK 0x7u
#define R27_EVENT2_HIGHLAND_ZONE_R_SHIFT 16 /* bit16-17 R 高地控制区激活(RMUL) */
#define R27_EVENT2_HIGHLAND_ZONE_R_MASK 0x3u
#define R27_EVENT2_HIGHLAND_ZONE_R_CTRL_SHIFT 18 /* bit18-19 0中立 1己方 2对方(RMUL) */
#define R27_EVENT2_HIGHLAND_ZONE_R_CTRL_MASK 0x3u
#define R27_EVENT2_HIGHLAND_ZONE_B_SHIFT 20
#define R27_EVENT2_HIGHLAND_ZONE_B_MASK 0x3u
#define R27_EVENT2_HIGHLAND_ZONE_B_CTRL_SHIFT 22
#define R27_EVENT2_HIGHLAND_ZONE_B_CTRL_MASK 0x3u
#define R27_EVENT2_CENTER_ZONE_SHIFT 24 /* bit24-25 中央控制区激活(RMUL) */
#define R27_EVENT2_CENTER_ZONE_MASK 0x3u
#define R27_EVENT2_CENTER_ZONE_CTRL_SHIFT 26 /* bit26-27 中央控制区控制状态(RMUL) */
#define R27_EVENT2_CENTER_ZONE_CTRL_MASK 0x3u

/* 位域取值辅助宏: R27_EVENT_GET(event_data, R27_EVENT_FORT_R) */
#define R27_EVENT_GET(word, field) \
    (((uint32_t)(word) >> field##_SHIFT) & field##_MASK)

/* ID: 0x0104  Byte: 3  裁判警告数据 (表 1-10) */
typedef struct
{
    uint8_t level;              /* 1双方黄牌 2黄牌 3红牌 4判负 */
    uint8_t offending_robot_id; /* 违规机器人 ID, 判负/双方黄牌时为 0 */
    uint8_t count;              /* 该判罚等级的违规次数, 开局为 0 */
} r27_referee_warning_t;

/* ID: 0x0105  Byte: 3  飞镖发射相关数据 (表 1-11) */
typedef struct
{
    uint8_t dart_remaining_time; /* 己方飞镖发射剩余时间, s */
    uint16_t dart_info;          /* bit0-2 最近一次击中的目标; bit3-5 对方累计被击中次数(<=4);
                                  * bit6-8 飞镖此时选定的击打目标; bit9-15 保留 */
} r27_dart_info_t;

#define R27_DART_HIT_TARGET_SHIFT 0
#define R27_DART_HIT_TARGET_MASK 0x7u
#define R27_DART_ENEMY_HIT_COUNT_SHIFT 3
#define R27_DART_ENEMY_HIT_COUNT_MASK 0x7u
#define R27_DART_SELECT_TARGET_SHIFT 6
#define R27_DART_SELECT_TARGET_MASK 0x7u

/* ======================== 常规链路: 主控模块 <-> 机器人 ======================== */

/* ID: 0x0201  Byte: 17  机器人性能体系数据 (表 1-12)
 * 相比旧协议新增 float bullet_speed_limit (offset 12) */
typedef struct
{
    uint8_t robot_id;                       /* 本机器人 ID */
    uint8_t robot_level;                    /* 机器人等级 */
    uint16_t current_HP;                    /* 当前血量 */
    uint16_t maximum_HP;                    /* 血量上限 */
    uint16_t shooter_barrel_cooling_value;  /* 射击热量每秒冷却值 */
    uint16_t shooter_barrel_heat_limit;     /* 射击热量上限 */
    uint16_t chassis_power_limit;           /* 底盘功率上限 */
    float bullet_speed_limit;               /* 射击初速度上限(新增) */
    uint8_t power_management_gimbal_output : 1;  /* bit0: gimbal 口 24V 输出 */
    uint8_t power_management_chassis_output : 1; /* bit1: chassis 口 24V 输出 */
    uint8_t power_management_shooter_output : 1; /* bit2: shooter 口 24V 输出 */
} r27_robot_status_t;

/* ID: 0x0202  Byte: 14  实时底盘缓冲能量和射击热量数据 (表 1-13)
 * 旧协议的 chassis_voltage / chassis_current / chassis_power 已被删除, 前 8 字节为保留位 */
typedef struct
{
    uint16_t reserved;                 /* 保留位 */
    uint16_t reserved_2;               /* 保留位 */
    float reserved_3;                  /* 保留位 */
    uint16_t buffer_energy;            /* 缓冲能量, 单位 J */
    uint16_t shooter_17mm_barrel_heat; /* 17mm 发射机构射击热量 */
    uint16_t shooter_42mm_barrel_heat; /* 42mm 发射机构射击热量 */
} r27_power_heat_data_t;

/* ID: 0x0203  Byte: 12  机器人位置数据 (表 1-14)
 * 旧协议的 z / yaw 已改为只有 angle(单位: 度, 正北为 0) */
typedef struct
{
    float x;     /* 位置 x, 单位 m */
    float y;     /* 位置 y, 单位 m */
    float angle; /* 测速模块朝向, 单位 度, 正北为 0 */
} r27_robot_pos_t;

/* ID: 0x0204  Byte: 7  机器人增益数据 (表 1-15)
 * cooling_buff 由 uint8 变为 uint16, 字段顺序也随之变化 */
typedef struct
{
    uint8_t recovery_buff;      /* 回血增益(百分比) */
    uint16_t cooling_buff;      /* 射击热量冷却增益具体值(直接值, x 表示 +x/s) */
    uint8_t defence_buff;       /* 防御增益(百分比) */
    uint8_t vulnerability_buff; /* 负防御增益(百分比) */
    uint16_t attack_buff;       /* 攻击增益(百分比) */
} r27_buff_t;

/* ID: 0x0206  Byte: 7  伤害状态数据 (表 1-16)
 * 新增命中序号与命中位置(单位 mm), 坐标系原点为装甲检测面左下角 */
typedef struct
{
    uint8_t armor_id : 4;          /* bit0-3: 装甲/测速模块 ID, 其他扣血原因时为 0 */
    uint8_t HP_deduction_reason : 4;/* bit4-7: 0弹丸 1装甲离线 2电容模块离线 5撞击 */
    uint16_t hit_sequence;         /* 命中事件序号, 每发 +1 */
    int16_t hit_position_x;        /* 命中位置 x, mm */
    int16_t hit_position_y;        /* 命中位置 y, mm */
} r27_hurt_data_t;

/* ID: 0x0207  Byte: 7  实时射击数据 (表 1-17) */
typedef struct
{
    uint8_t bullet_type;          /* 1: 17mm 弹丸 / 2: 42mm 弹丸 */
    uint8_t shooter_number;       /* 1: 17mm 发射机构 / 3: 42mm 发射机构 */
    uint8_t launching_frequency;  /* 弹丸射速, 单位 Hz */
    float initial_speed;          /* 弹丸初速度, 单位 m/s */
} r27_shoot_data_t;

/* ======================== 常规链路: 服务器 -> 特定机器人 ======================== */

/* ID: 0x0208  Byte: 8  允许发弹量与剩余金币 (表 1-18) */
typedef struct
{
    uint16_t projectile_allowance_17mm;    /* 自身 17mm 允许发弹量 */
    uint16_t projectile_allowance_42mm;    /* 42mm 允许发弹量 */
    uint16_t remaining_gold_coin;          /* 剩余金币 */
    uint16_t projectile_allowance_fortress;/* 堡垒增益点提供的储备允许发弹量 */
} r27_projectile_allowance_t;

/* ID: 0x0209  Byte: 6  机器人 RFID 模块状态 (表 1-19)
 * rfid_status  bit0己方补给区 bit1己方装配区 bit2 R基地 bit3 B基地 bit4 R堡垒 bit5 B堡垒
 *              bit6 R中央高地 bit7 B中央高地 bit8 R前哨 bit9 B前哨 bit10 己方制高点
 *              bit11 R高地控制区(RMUL) bit12 B高地控制区(RMUL) bit13 中央控制区(RMUL)
 *              bit14/15 己方地形跨越(中央高地下方/上方) bit16/17 对方...
 *              bit18~21 己方/对方公路下方/上方 bit22~30 己方隧道各增益点
 * rfid_status_2 bit0~8 对方隧道各增益点
 */
typedef struct
{
    uint32_t rfid_status;
    uint16_t rfid_status_2;
} r27_rfid_status_t;

/* ID: 0x020A  Byte: 6  飞镖选手端指令数据 (表 1-20) */
typedef struct
{
    uint8_t dart_launch_opening_status; /* 1关闭 2开启/关闭中 0已开启 */
    uint8_t reserved;                   /* 保留位 */
    uint16_t target_change_time;        /* 切换击打目标时的比赛剩余时间, s */
    uint16_t latest_launch_cmd_time;    /* 最后一次确认发射指令时的比赛剩余时间, s */
} r27_dart_client_cmd_t;

/* ID: 0x020B  Byte: 40  地面机器人位置数据 (表 1-21, 发送给哨兵)
 * 坐标系: 红方补给站附近围挡交点为原点, 沿长边向蓝方为 X+, 沿短边向红方停机坪为 Y+ */
typedef struct
{
    float heavyengineer_x;   /* 己方重装 x, m */
    float heavyengineer_y;   /* 己方重装 y, m */
    float reserved_1;
    float reserved_2;
    float standard_3_x;      /* 己方 3 号步兵 x, m */
    float standard_3_y;
    float standard_4_x;      /* 己方 4 号步兵 x, m */
    float standard_4_y;
    float reserved_3;
    float reserved_4;
} r27_ground_robot_position_t;

/* ID: 0x020C  Byte: 2  雷达标记进度数据 (表 1-22)
 * bit0 对方重装易伤 bit1 保留 bit2 对方3号易伤 bit3 对方4号易伤 bit4 保留 bit5 对方哨兵易伤
 * bit6 己方重装特殊标识 bit7 保留 bit8 己方3号 bit9 己方4号 bit10 保留 bit11 己方哨兵
 * bit12~15 保留
 * (对方标记进度>=100 时置 1; 己方标记进度>=50 时置 1) */
typedef struct
{
    uint16_t mark_progress;
} r27_radar_mark_data_t;

/* ID: 0x020D  Byte: 14  哨兵自主决策信息同步 (表 1-23) */
typedef struct
{
    uint32_t sentry_info;   /* bit0-10 已兑换允许发弹量; bit11-14 远程兑换发弹量次数;
                             * bit15-18 远程兑换血量次数; bit19 可确认免费复活; bit20 可兑换立即复活;
                             * bit21-30 立即复活所需金币; bit31 保留 */
    uint16_t sentry_info_2; /* bit0 脱战状态; bit1-11 队伍 17mm 剩余可兑换数;
                             * bit12-13 当前姿态(1进攻 2防御 3移动); bit14 能量机关可激活;
                             * bit15 是否强化姿态 */
    uint64_t sentry_info_3; /* bit0-7/8-15/16-23 进攻/防御/移动姿态弱化前剩余可持续时长(s);
                             * bit32-39/40-47/48-55 强化进攻/防御/移动姿态剩余可持续时长 */
} r27_sentry_info_t;

/* ID: 0x020E  Byte: 8  雷达自主决策信息同步 (表 1-24) */
typedef struct
{
    uint8_t radar_info;    /* bit0-1 可触发双倍易伤次数(<=2); bit2 对方是否正被双倍易伤;
                            * bit3-4 己方加密等级(1~3); bit5 当前是否可修改密钥 */
    uint8_t radar_info_2;  /* bit0 是否存在生效中的装甲加固区域; bit1-3 剩余装甲加固机会数 */
    uint16_t fortify_center_x; /* 加固区圆心 x, mm */
    uint16_t fortify_center_y; /* 加固区圆心 y, mm */
    uint16_t fortify_radius;   /* 加固区半径, mm */
} r27_radar_info_t;

/* ======================== 常规链路: 小地图交互数据 ======================== */

/* ID: 0x0303  Byte: 12  选手端小地图下发数据 (表 1-35)
 * 触发后服务器以 100ms 间隔补发 4 次(共 5 次), 之后以 1Hz 定频发送最近一次的包 */
typedef struct
{
    float target_position_x;   /* 目标位置 x, m; 发送目标机器人 ID 时为 0 */
    float target_position_y;   /* 目标位置 y, m; 同上 */
    uint8_t cmd_keyboard;      /* 云台手按下的键盘通用键值, 无按键为 0 */
    uint8_t target_robot_id;   /* 对方机器人 ID, 发送坐标数据时为 0 */
    uint16_t cmd_source;       /* 信息来源 ID */
} r27_map_command_t;

/* ID: 0x0305  Byte: 48  选手端小地图接收雷达数据 (表 1-36), 单位 cm */
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
    uint16_t ally_heavy_position_x;
    uint16_t ally_heavy_position_y;
    uint16_t reserved_5;
    uint16_t reserved_6;
    uint16_t ally_infantry_3_position_x;
    uint16_t ally_infantry_3_position_y;
    uint16_t ally_infantry_4_position_x;
    uint16_t ally_infantry_4_position_y;
    uint16_t reserved_7;
    uint16_t reserved_8;
    uint16_t ally_sentry_position_x;
    uint16_t ally_sentry_position_y;
} r27_map_robot_data_t;

/* ID: 0x0307  Byte: 105  选手端小地图接收路径数据 (表 1-37), 单位 dm
 * 小地图左下角为原点, 向右为 X+, 向上为 Y+, 增量为相对上一点的增量 */
typedef struct
{
    uint8_t intention;         /* 1到目标点攻击 2到目标点防守 3移动到目标点 */
    uint16_t start_position_x; /* 路径起点 x */
    uint16_t start_position_y; /* 路径起点 y */
    int8_t delta_x[49];        /* 路径点 x 增量数组 */
    int8_t delta_y[49];        /* 路径点 y 增量数组 */
    uint16_t sender_id;        /* 发送者 ID, 需与自身 ID 匹配 */
} r27_map_data_t;

/* ID: 0x0308  Byte: 34  选手端小地图接收机器人数据(自定义消息) (表 1-38) */
typedef struct
{
    uint16_t sender_id;   /* 发送者 ID */
    uint16_t receiver_id; /* 接收者 ID, 仅支持发给己方选手端 */
    uint8_t user_data[30];/* 字符, 以 UTF-16 编码, 支持中文(注意大小端) */
} r27_custom_info_t;

/* ======================== 图传链路数据 ======================== */

/* ID: 0x0310  Byte: 300  机器人 -> 自定义终端 (表 1-39), 上限 50Hz, 无重传 */
typedef struct
{
    uint8_t data[300];
} r27_robot_custom_data_t;

/* ID: 0x0311  Byte: 60  自定义终端 -> 机器人 (表 1-40), 上限 75Hz, 无重传
 * 内容为 KeyboardMouseControl 或 CustomControl(<=60 字节自定义数据) */
typedef struct
{
    uint8_t data[60];
} r27_custom_terminal_cmd_t;

/* ======================== 0x0301 机器人交互数据 ======================== */
/* 数据段 = 子内容头(6B) + 内容数据段(x, x<=112), 包总长 <= 127 字节 */

/* 交互数据头结构 (6 字节) */
typedef struct
{
    uint16_t data_cmd_id; /* 子内容 ID */
    uint16_t sender_id;   /* 发送者 ID */
    uint16_t receiver_id; /* 接收者 ID */
} r27_interaction_header_t;

/* 子内容 ID (表 1-25) */
typedef enum
{
    /* 机器人之间通信, 区间 0x0200~0x02FF, 内容长度 x <= 112 */
    R27_SubID_RobotComm = 0x0200,

    /* 选手端 UI */
    R27_SubID_UI_LayerDelete = 0x0100, /* 内容长度 2 */
    R27_SubID_UI_Draw1 = 0x0101,       /* 内容长度 15 */
    R27_SubID_UI_Draw2 = 0x0102,       /* 内容长度 30 */
    R27_SubID_UI_Draw5 = 0x0103,       /* 内容长度 75 */
    R27_SubID_UI_Draw7 = 0x0104,       /* 内容长度 105 */
    R27_SubID_UI_DrawChar = 0x0110,    /* 内容长度 45 */

    /* 自主决策指令, 发送者 -> 0x8080(裁判系统服务器) */
    R27_SubID_SentryCmd = 0x0120,      /* 内容长度 4 */
    R27_SubID_RadarCmd = 0x0121,       /* 内容长度 13 */
} R27Interactive_Data_ID_e;

/* 子内容长度 */
typedef enum
{
    R27_INTERACTION_LEN_LayerDelete = 2,
    R27_INTERACTION_LEN_Draw1 = 15,
    R27_INTERACTION_LEN_Draw2 = 30,
    R27_INTERACTION_LEN_Draw5 = 75,
    R27_INTERACTION_LEN_Draw7 = 105,
    R27_INTERACTION_LEN_DrawChar = 45,
    R27_INTERACTION_LEN_SentryCmd = 4,
    R27_INTERACTION_LEN_RadarCmd = 13,
} R27Interactive_Data_Length_e;

/* 子内容 ID: 0x0100 删除图层 (表 1-26) */
typedef struct
{
    uint8_t delete_type; /* 0 空操作 / 1 删除图层 / 2 删除所有 */
    uint8_t layer;       /* 图层数 0~9 */
} r27_interaction_layer_delete_t;

/* 子内容 ID: 0x0101 绘制一个图形 (表 1-27)
 * 15 字节, 与旧协议的 Graph_Data_t 位布局完全一致, 仅字段改名:
 *   start_angle -> details_a, end_angle -> details_b, radius -> details_c,
 *   end_x       -> details_d, end_y     -> details_e
 *
 * 图形细节参数 (表 1-28):
 *   直线   : details_d/e = 终点 x/y
 *   矩形   : details_d/e = 对角顶点 x/y
 *   正圆   : details_c = 半径
 *   椭圆   : details_d/e = x/y 半轴长度
 *   圆弧   : details_a/b = 起始/终止角度, details_d/e = x/y 半轴长度
 *   浮点数 : details_a = 字体大小, details_b 无作用, details_c/d/e 组成 32 位值(值/1000 为显示值)
 *   整型数 : details_a = 字体大小, details_c/d/e 组成 32 位 int32_t
 *   字符   : details_a = 字体大小, details_b = 字符长度
 * 角度: 0° 为 12 点钟方向, 顺时针绘制; 屏幕 (0,0) 为左下角, (1920,1080) 为右上角。 */
typedef struct
{
    uint8_t figure_name[3]; /* 图形名, 作为删除/修改的索引 */
    uint32_t operate_tpye : 3; /* bit0-2  图形操作: 0空 1增加 2修改 3删除 */
    uint32_t figure_tpye : 3;  /* bit3-5  图形类型: 0直线 1矩形 2正圆 3椭圆 4圆弧 5浮点 6整型 7字符 */
    uint32_t layer : 4;        /* bit6-9  图层数 0~9 */
    uint32_t color : 4;        /* bit10-13 颜色 0红/蓝(己方) 1黄 2绿 3橙 4紫红 5粉 6青 7黑 8白 */
    uint32_t details_a : 9;    /* bit14-22 细节参数 a */
    uint32_t details_b : 9;    /* bit23-31 细节参数 b */
    uint32_t width : 10;       /* 线宽, 建议字体大小:线宽 = 10:1 */
    uint32_t start_x : 11;     /* 起点/圆心 x */
    uint32_t start_y : 11;     /* 起点/圆心 y */
    uint32_t details_c : 10;   /* 细节参数 c */
    uint32_t details_d : 11;   /* 细节参数 d */
    uint32_t details_e : 11;   /* 细节参数 e */
} r27_interaction_figure_t;

/* 子内容 ID: 0x0102 绘制两个图形 (表 1-29) */
typedef struct
{
    r27_interaction_figure_t figure[2];
} r27_interaction_figure_2_t;

/* 子内容 ID: 0x0103 绘制五个图形 (表 1-30) */
typedef struct
{
    r27_interaction_figure_t figure[5];
} r27_interaction_figure_5_t;

/* 子内容 ID: 0x0104 绘制七个图形 (表 1-31) */
typedef struct
{
    r27_interaction_figure_t figure[7];
} r27_interaction_figure_7_t;

/* 子内容 ID: 0x0110 绘制字符 (表 1-32) */
typedef struct
{
    r27_interaction_figure_t figure; /* 字符配置, 15 字节 */
    uint8_t data[30];                /* 字符内容 */
} r27_interaction_string_t;

/* 子内容 ID: 0x0120 哨兵自主决策指令 (表 1-33), 接收者为 0x8080 */
typedef struct
{
    uint32_t sentry_cmd; /* bit0 确认复活; bit1 确认兑换立即复活;
                          * bit2-12 将要兑换的发弹量值(单调递增);
                          * bit13-16 远程兑换发弹量请求次数(每次 +1);
                          * bit17-20 远程兑换血量请求次数(每次 +1);
                          * bit21-23 修改姿态: 1进攻 2防御 3移动 4强化进攻 5强化防御 6强化移动;
                          * bit24 确认使能量机关进入正在激活状态; bit25-31 保留 */
} r27_sentry_cmd_t;

#define R27_SENTRY_CMD_CONFIRM_RESPAWN_SHIFT 0
#define R27_SENTRY_CMD_EXCHANGE_RESPAWN_SHIFT 1
#define R27_SENTRY_CMD_EXCHANGE_AMMO_SHIFT 2
#define R27_SENTRY_CMD_REMOTE_AMMO_SHIFT 13
#define R27_SENTRY_CMD_REMOTE_HEAL_SHIFT 17
#define R27_SENTRY_CMD_POSTURE_SHIFT 21
#define R27_SENTRY_CMD_RUNE_ACTIVATE_SHIFT 24

/* 子内容 ID: 0x0121 雷达自主决策指令 (表 1-34), 接收者为 0x8080 */
typedef struct
{
    uint8_t radar_cmd;          /* 请求触发双倍易伤(单调递增, 每次 +1) */
    uint8_t password_cmd;       /* 密钥指令类型: 0无 / 1更新己方密钥 / 2验证破解的对方密钥 */
    uint8_t password_1;         /* 密钥值 byte1(ASCII) */
    uint8_t password_2;
    uint8_t password_3;
    uint8_t password_4;
    uint8_t password_5;
    uint8_t password_6;         /* 密钥值 byte6(ASCII) */
    uint8_t armor_fortify_cmd;  /* 请求使用装甲加固机会(单调递增, 每次 +1) */
    uint16_t fortify_center_x;  /* 加固区圆心 x, mm */
    uint16_t fortify_center_y;  /* 加固区圆心 y, mm */
} r27_radar_cmd_t;

/* ======================== UI 相关枚举 ======================== */

/* 图形配置参数__删除操作 */
typedef enum
{
    R27_UI_Del_NoOperate = 0,
    R27_UI_Del_Layer = 1,
    R27_UI_Del_ALL = 2, /* 删除全部图层, 后续参数不重要 */
} R27_UI_Delete_Operate_e;

/* 图形配置参数__图形操作 */
typedef enum
{
    R27_UI_Graph_ADD = 1,
    R27_UI_Graph_Change = 2,
    R27_UI_Graph_Del = 3,
} R27_UI_Graph_Operate_e;

/* 图形配置参数__图形类型 */
typedef enum
{
    R27_UI_Graph_Line = 0,      /* 直线 */
    R27_UI_Graph_Rectangle = 1, /* 矩形 */
    R27_UI_Graph_Circle = 2,    /* 正圆 */
    R27_UI_Graph_Ellipse = 3,   /* 椭圆 */
    R27_UI_Graph_Arc = 4,       /* 圆弧 */
    R27_UI_Graph_Float = 5,     /* 浮点型 */
    R27_UI_Graph_Int = 6,       /* 整形 */
    R27_UI_Graph_Char = 7,      /* 字符型 */
} R27_UI_Graph_Type_e;

/* 图形配置参数__图形颜色 */
typedef enum
{
    R27_UI_Color_Main = 0, /* 红/蓝(己方颜色) */
    R27_UI_Color_Yellow = 1,
    R27_UI_Color_Green = 2,
    R27_UI_Color_Orange = 3,
    R27_UI_Color_Purplish_red = 4, /* 紫红色 */
    R27_UI_Color_Pink = 5,
    R27_UI_Color_Cyan = 6, /* 青色 */
    R27_UI_Color_Black = 7,
    R27_UI_Color_White = 8,
} R27_UI_Graph_Color_e;

/* ======================== ID 编号 (附录二) ======================== */

/* 机器人 ID */
typedef enum
{
    /* 红方 */
    R27_RobotID_RHeavy = 1,      /* 红方重装机器人 */
    R27_RobotID_RReserved = 2,   /* 保留位(旧协议的工程机器人已取消) */
    R27_RobotID_RStandard_3 = 3, /* 红方 3 号步兵 */
    R27_RobotID_RStandard_4 = 4, /* 红方 4 号步兵 */
    R27_RobotID_RStandard_5 = 5, /* 红方 5 号步兵 */
    R27_RobotID_RAerial = 6,     /* 红方空中系统 */
    R27_RobotID_RSentry = 7,     /* 红方哨兵 */
    R27_RobotID_RDart = 8,       /* 红方飞镖 */
    R27_RobotID_RRadar = 9,      /* 红方雷达 */
    R27_RobotID_RBase = 10,      /* 红方基地 */
    R27_RobotID_ROutpost = 11,   /* R 前哨 */
    /* 蓝方 = 红方 + 100 */
    R27_RobotID_BHeavy = 101,
    R27_RobotID_BReserved = 102,
    R27_RobotID_BStandard_3 = 103,
    R27_RobotID_BStandard_4 = 104,
    R27_RobotID_BStandard_5 = 105,
    R27_RobotID_BAerial = 106,
    R27_RobotID_BSentry = 107,
    R27_RobotID_BDart = 108,
    R27_RobotID_BRadar = 109,
    R27_RobotID_BBase = 110,
    R27_RobotID_BOutpost = 111,
} R27_Robot_ID_e;

/* 选手端(客户端) ID, 只有重装/步兵/空中系统有选手端 */
typedef enum
{
    R27_ClientID_RHeavy = 0x0101,
    R27_ClientID_RReserved = 0x0102,
    R27_ClientID_RStandard_3 = 0x0103,
    R27_ClientID_RStandard_4 = 0x0104,
    R27_ClientID_RStandard_5 = 0x0105,
    R27_ClientID_RAerial = 0x0106,

    R27_ClientID_BHeavy = 0x0165,
    R27_ClientID_BReserved = 0x0166,
    R27_ClientID_BStandard_3 = 0x0167,
    R27_ClientID_BStandard_4 = 0x0168,
    R27_ClientID_BStandard_5 = 0x0169,
    R27_ClientID_BAerial = 0x016A,

    R27_ClientID_Server = 0x8080, /* 裁判系统服务器(哨兵/雷达自主决策指令) */
} R27_Client_ID_e;

/* ======================== 编译期长度校验 ======================== */
/* 手册更新导致长度变化时, 这里会直接编译失败, 避免收包解析悄悄错位 */

_Static_assert(sizeof(r27_frame_header_t) == 5, "frame_header must be 5 bytes");
_Static_assert(sizeof(r27_game_status_t) == R27_LEN_game_state, "0x0001 length mismatch");
_Static_assert(sizeof(r27_game_result_t) == R27_LEN_game_result, "0x0002 length mismatch");
_Static_assert(sizeof(r27_game_robot_HP_t) == R27_LEN_game_robot_HP, "0x0003 length mismatch");
_Static_assert(sizeof(r27_event_data_t) == R27_LEN_event_data, "0x0101 length mismatch");
_Static_assert(sizeof(r27_referee_warning_t) == R27_LEN_referee_warning, "0x0104 length mismatch");
_Static_assert(sizeof(r27_dart_info_t) == R27_LEN_dart_info, "0x0105 length mismatch");
_Static_assert(sizeof(r27_robot_status_t) == R27_LEN_robot_status, "0x0201 length mismatch");
_Static_assert(sizeof(r27_power_heat_data_t) == R27_LEN_power_heat_data, "0x0202 length mismatch");
_Static_assert(sizeof(r27_robot_pos_t) == R27_LEN_robot_pos, "0x0203 length mismatch");
_Static_assert(sizeof(r27_buff_t) == R27_LEN_robot_buff, "0x0204 length mismatch");
_Static_assert(sizeof(r27_hurt_data_t) == R27_LEN_hurt_data, "0x0206 length mismatch");
_Static_assert(sizeof(r27_shoot_data_t) == R27_LEN_shoot_data, "0x0207 length mismatch");
_Static_assert(sizeof(r27_projectile_allowance_t) == R27_LEN_projectile_allowance, "0x0208 length mismatch");
_Static_assert(sizeof(r27_rfid_status_t) == R27_LEN_rfid_status, "0x0209 length mismatch");
_Static_assert(sizeof(r27_dart_client_cmd_t) == R27_LEN_dart_client_cmd, "0x020A length mismatch");
_Static_assert(sizeof(r27_ground_robot_position_t) == R27_LEN_ground_robot_position, "0x020B length mismatch");
_Static_assert(sizeof(r27_radar_mark_data_t) == R27_LEN_radar_mark_data, "0x020C length mismatch");
_Static_assert(sizeof(r27_sentry_info_t) == R27_LEN_sentry_info, "0x020D length mismatch");
_Static_assert(sizeof(r27_radar_info_t) == R27_LEN_radar_info, "0x020E length mismatch");
_Static_assert(sizeof(r27_map_command_t) == R27_LEN_map_command, "0x0303 length mismatch");
_Static_assert(sizeof(r27_map_robot_data_t) == R27_LEN_map_robot_data, "0x0305 length mismatch");
_Static_assert(sizeof(r27_map_data_t) == R27_LEN_map_data, "0x0307 length mismatch");
_Static_assert(sizeof(r27_custom_info_t) == R27_LEN_custom_info, "0x0308 length mismatch");
_Static_assert(sizeof(r27_robot_custom_data_t) == R27_LEN_robot_custom_data, "0x0310 length mismatch");
_Static_assert(sizeof(r27_custom_terminal_cmd_t) == R27_LEN_custom_terminal_cmd, "0x0311 length mismatch");
_Static_assert(sizeof(r27_interaction_header_t) == R27_INTERACTION_HEAD_LEN, "interaction header must be 6 bytes");
_Static_assert(sizeof(r27_interaction_figure_t) == R27_INTERACTION_LEN_Draw1, "figure must be 15 bytes");
_Static_assert(sizeof(r27_interaction_figure_2_t) == R27_INTERACTION_LEN_Draw2, "figure x2 mismatch");
_Static_assert(sizeof(r27_interaction_figure_5_t) == R27_INTERACTION_LEN_Draw5, "figure x5 mismatch");
_Static_assert(sizeof(r27_interaction_figure_7_t) == R27_INTERACTION_LEN_Draw7, "figure x7 mismatch");
_Static_assert(sizeof(r27_interaction_string_t) == R27_INTERACTION_LEN_DrawChar, "char figure must be 45 bytes");
_Static_assert(sizeof(r27_interaction_layer_delete_t) == R27_INTERACTION_LEN_LayerDelete, "delete must be 2 bytes");
_Static_assert(sizeof(r27_sentry_cmd_t) == R27_INTERACTION_LEN_SentryCmd, "sentry cmd must be 4 bytes");
_Static_assert(sizeof(r27_radar_cmd_t) == R27_INTERACTION_LEN_RadarCmd, "radar cmd must be 13 bytes");

/* 关键字段偏移校验(抽查手册中的 offset) */
_Static_assert(offsetof(r27_robot_status_t, bullet_speed_limit) == 12, "0x0201 bullet_speed_limit offset");
_Static_assert(sizeof(r27_robot_status_t) - offsetof(r27_robot_status_t, bullet_speed_limit) == 5,
               "0x0201 power management byte must follow bullet_speed_limit");
_Static_assert(offsetof(r27_game_robot_HP_t, damage_difference) == 8, "0x0003 damage_difference offset");
_Static_assert(offsetof(r27_power_heat_data_t, buffer_energy) == 8, "0x0202 buffer_energy offset");
_Static_assert(offsetof(r27_buff_t, cooling_buff) == 1, "0x0204 cooling_buff offset");
_Static_assert(offsetof(r27_sentry_info_t, sentry_info_3) == 6, "0x020D sentry_info_3 offset");
_Static_assert(offsetof(r27_radar_cmd_t, fortify_center_x) == 9, "0x0121 fortify_center_x offset");
_Static_assert(offsetof(r27_map_data_t, delta_x) == 5, "0x0307 delta_x offset");
_Static_assert(offsetof(r27_map_data_t, delta_y) == 54, "0x0307 delta_y offset");

#pragma pack()

#endif /* REFEREE27_PROTOCOL_H */
