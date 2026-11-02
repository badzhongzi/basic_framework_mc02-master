/**
 * @file rm_referee27.h
 * @brief 裁判系统 2027 协议通信模块对外接口
 *
 * @note 与旧模块 modules/referee 的区别:
 *       1. 支持多链路: 常规链路(电源管理模块 User 串口 115200) /
 *          图传链路(图传发送端串口 921600) / 雷达链路(雷达接收机, 单向),
 *          每个链路一个 R27Link 实例, 解析时按链路过滤命令码, 互不干扰;
 *       2. 0x0301 交互数据支持变长接收(旧实现固定拷贝 11 字节, 会把 118 字节的包截断);
 *       3. 所有结构体与 2027 V1.0.0 手册逐字节对应, 并有编译期长度断言;
 *       4. 与旧模块的符号完全隔离(R27_ / r27_ 前缀), 可同时编译, 便于逐步迁移。
 */

#ifndef RM_REFEREE27_H
#define RM_REFEREE27_H

#include "stdint.h"
#include "main.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "referee27_protocol.h"
#include "radar27_protocol.h"
#include "robot_def.h"

/* ============================== 链路定义 ============================== */

typedef enum
{
    R27_LINK_COMMON = 0, /* 常规链路: 电源管理模块 User 串口, 115200 */
    R27_LINK_VIDEO,      /* 图传链路: 图传发送端串口, 921600 */
    R27_LINK_RADAR,      /* 雷达链路: 雷达接收机(串口转发), 单向 */
    R27_LINK_TYPE_CNT
} R27LinkType_e;

/* 发送节流间隔(ms)。手册规定 0x0301 上行上限 30Hz, 但裁判系统接收额度有限
 * (重装/步兵 3720B/s, 哨兵/雷达 5120B/s), UI 刷新并不需要太高频率, 默认取 100ms(10Hz)。
 * 若需要提高 UI 刷新率, 请先核算带宽后再改小。 */
#define R27_TX_MIN_INTERVAL_MS 100

/* 常规链路接收缓冲大小(0x0301 单包最大 127 字节, 一次 DMA 完成可能包含多包数据)
 * @note bsp_usart 的 USART_RXBUFF_LIMIT 为 256, recv_buff_size 为 uint8_t, 故上限 255。
 *       图传链路只需接收 0x0311(69 字节/包), 雷达链路单包最大 56 字节, 均无需扩大。 */
#define R27_COMMON_RX_BUFFER_SIZE 255u
#define R27_VIDEO_RX_BUFFER_SIZE 128u
#define R27_RADAR_RX_BUFFER_SIZE 128u

/* ============================== 运行时数据 ============================== */

/* 本机 ID 信息(UI 绘制与机器人间通信使用) */
typedef struct
{
    uint8_t Robot_Color;        /* R27_ROBOT_RED / R27_ROBOT_BLUE */
    uint16_t Robot_ID;          /* 本机器人 ID */
    uint16_t Client_ID;         /* 本机器人对应的选手端 ID (0x0100 + Robot_ID) */
    uint16_t Receiver_Robot_ID; /* 机器人间通信时的接收者 ID, 必须和本机器人同颜色 */
} r27_referee_id_t;

/* 0x0301 机器人交互数据(变长解析结果) */
typedef struct
{
    uint16_t data_cmd_id; /* 子内容 ID */
    uint16_t sender_id;
    uint16_t receiver_id;
    uint16_t data_len; /* 实际收到的内容数据段长度(不含 6 字节子内容头) */
    uint8_t data[R27_INTERACTION_DATA_MAX];
} r27_robot_interaction_t;

/* 常规链路 + 图传链路 接收数据 */
typedef struct
{
    r27_frame_header_t FrameHeader; /* 最近一帧的帧头 */
    uint16_t CmdID;                 /* 最近一帧的命令码 */

    /* ---- 服务器 -> 全体机器人 ---- */
    r27_game_status_t GameState;          /* 0x0001 比赛状态 */
    r27_game_result_t GameResult;         /* 0x0002 比赛结果 */
    r27_game_robot_HP_t GameRobotHP;      /* 0x0003 血量/伤害差/控制度 */
    r27_event_data_t EventData;           /* 0x0101 场地事件 */
    r27_referee_warning_t RefereeWarning; /* 0x0104 裁判警告 */
    r27_dart_info_t DartInfo;             /* 0x0105 飞镖数据 */

    /* ---- 主控模块 <-> 本机器人 ---- */
    r27_robot_status_t RobotStatus;       /* 0x0201 性能体系 */
    r27_power_heat_data_t PowerHeatData;  /* 0x0202 缓冲能量/热量 */
    r27_robot_pos_t RobotPos;             /* 0x0203 位置 */
    r27_buff_t Buff;                      /* 0x0204 增益 */
    r27_hurt_data_t HurtData;             /* 0x0206 伤害状态 */
    r27_shoot_data_t ShootData;           /* 0x0207 实时射击 */

    /* ---- 服务器 -> 特定机器人 ---- */
    r27_projectile_allowance_t ProjectileAllowance; /* 0x0208 允许发弹量/金币 */
    r27_rfid_status_t RfidStatus;                   /* 0x0209 RFID */
    r27_dart_client_cmd_t DartClientCmd;            /* 0x020A 飞镖选手端指令 */
    r27_ground_robot_position_t GroundRobotPos;     /* 0x020B 地面机器人位置(哨兵) */
    r27_radar_mark_data_t RadarMarkData;            /* 0x020C 雷达标记进度 */
    r27_sentry_info_t SentryInfo;                   /* 0x020D 哨兵决策同步 */
    r27_radar_info_t RadarInfo;                          /* 0x020E 雷达决策同步 */

    /* ---- 小地图 ---- */
    r27_map_command_t MapCommand;         /* 0x0303 选手端下发 */
    r27_map_robot_data_t MapRobotData;    /* 0x0305 雷达给选手端 */
    r27_map_data_t MapData;               /* 0x0307 路径数据 */
    r27_custom_info_t CustomInfo;         /* 0x0308 自定义消息 */

    /* ---- 机器人交互 ---- */
    r27_robot_interaction_t Interaction;  /* 0x0301 */

    /* ---- 图传链路 ---- */
    r27_custom_terminal_cmd_t TerminalCmd; /* 0x0311 自定义终端下发的 60 字节 */

    /* ---- 统计 ---- */
    uint32_t rx_frame_cnt;         /* 成功解析的帧数 */
    uint32_t rx_error_cnt;         /* 校验失败的帧数 */
    uint32_t rx_unknown_cnt;       /* 不属于本链路的帧数 */
    uint32_t rx_len_mismatch_cnt;  /* 数据段长度与手册不符的帧数(排查协议版本用) */
    uint16_t last_data_length;  /* 最近一帧的数据段长度 */
    uint8_t init_flag;          /* 已初始化(收到过数据)标志 */
} r27_referee_info_t;

/* 雷达链路(0x0A01~0x0A06) 接收数据 */
typedef struct
{
    r27_frame_header_t FrameHeader;
    uint16_t CmdID;

    r27_radar_robot_pos_t RobotPos;       /* 0x0A01 对方位置(cm) */
    r27_radar_robot_hp_t RobotHP;         /* 0x0A02 对方血量 */
    r27_radar_robot_ammo_t RobotAmmo;     /* 0x0A03 对方允许发弹量 */
    r27_radar_team_status_t TeamStatus;   /* 0x0A04 对方宏观状态 */
    r27_radar_robot_buff_t RobotBuff;     /* 0x0A05 对方增益/姿态/加固区 */
    r27_radar_key_t Key;                  /* 0x0A06 对方干扰波密钥 */

    uint32_t rx_frame_cnt;
    uint32_t rx_error_cnt;
    uint32_t rx_len_mismatch_cnt;
    uint8_t init_flag;
} r27_radar_link_info_t;

/* 单条链路的实例 */
typedef struct r27_link
{
    USARTInstance *usart;   /* 串口实例 */
    DaemonInstance *daemon; /* 守护进程, 用于掉线检测 */
    R27LinkType_e type;     /* 链路类型 */

    r27_referee_info_t info; /* 常规/图传链路数据 */
    r27_radar_link_info_t radar;  /* 雷达链路数据 */
} R27Link;

/* ===================== UI 绘制所需的非裁判系统数据 ===================== */

/* 模式是否切换标志位, 1 为已切换待刷新, 0 为未切换 */
typedef struct
{
    uint32_t chassis_flag : 1;
    uint32_t gimbal_flag : 1;
    uint32_t shoot_flag : 1;
    uint32_t lid_flag : 1;
    uint32_t friction_flag : 1;
    uint32_t Power_flag : 1;
} R27_Interactive_Flag_t;

typedef struct
{
    R27_Interactive_Flag_t Interactive_Flag;

    /* 待显示的数据(由上层应用填写) */
    chassis_mode_e chassis_mode;
    gimbal_mode_e gimbal_mode;
    shoot_mode_e shoot_mode;
    friction_mode_e friction_mode;
    lid_mode_e lid_mode;
    Chassis_Power_Data_s Chassis_Power_Data;

    /* 上一次的值, 用于变化检测 */
    chassis_mode_e chassis_last_mode;
    gimbal_mode_e gimbal_last_mode;
    shoot_mode_e shoot_last_mode;
    friction_mode_e friction_last_mode;
    lid_mode_e lid_last_mode;
    Chassis_Power_Data_s Chassis_last_Power_Data;
} r27_interactive_info_t;

/* ============================== 接口 ============================== */

/**
 * @brief 初始化一条裁判系统链路(注册串口 + 守护进程 + 开启 DMA 接收)
 *
 * @param type   链路类型, 见 R27LinkType_e
 * @param handle 该链路对应的串口句柄(常规: C 板通常为 USART6)
 * @return R27Link* 链路实例指针, 失败返回 NULL
 * @note 同一条链路重复调用会直接返回已注册的实例
 */
R27Link *R27RefereeInit(R27LinkType_e type, UART_HandleTypeDef *handle);

/**
 * @brief 获取已注册的链路实例
 * @return R27Link* 未注册返回 NULL
 */
R27Link *R27RefereeGetLink(R27LinkType_e type);

/**
 * @brief 获取链路接收到的数据
 */
r27_referee_info_t *R27RefereeGetInfo(R27LinkType_e type);

/**
 * @brief 获取雷达链路数据(仅 R27_LINK_RADAR 有值)
 */
r27_radar_link_info_t *R27RadarGetInfo(R27LinkType_e type);

/**
 * @brief 链路是否在线(daemon 判定)
 */
uint8_t R27RefereeIsOnline(R27LinkType_e type);

/**
 * @brief 获取下一个包序号(全局自增, 周期 0~255)
 */
uint8_t R27RefereeNextSeq(void);

/**
 * @brief 阻塞等待串口空闲后以 DMA 发送一帧(不额外延时)
 * @note 用于高频/大包发送(如图传 0x0310), 调用者自行控制频率
 */
void R27RefereeSend(R27Link *link, uint8_t *buf, uint16_t len);

/**
 * @brief 发送一帧并做节流(发送后挂起 R27_TX_MIN_INTERVAL_MS)
 * @note 用于 0x0301 交互数据(UI/多机通信), 避免占用过多带宽
 */
void R27RefereeSendThrottled(R27Link *link, uint8_t *buf, uint16_t len);

/* ============================== 工具函数 ============================== */

/* 机器人 ID -> 颜色 */
static inline uint8_t R27RobotColorOf(uint16_t robot_id)
{
    return (robot_id >= R27_ROBOT_ID_COLOR_SPLIT) ? R27_ROBOT_BLUE : R27_ROBOT_RED;
}

/* 机器人 ID -> 选手端 ID(仅 1~6 / 101~106 有选手端) */
static inline uint16_t R27ClientIdOf(uint16_t robot_id)
{
    return R27_CLIENT_ID_OF_ROBOT(robot_id);
}

#endif /* RM_REFEREE27_H */
