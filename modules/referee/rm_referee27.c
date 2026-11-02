/**
 * @file rm_referee27.c
 * @brief 裁判系统 2027 协议通信实现(多链路: 常规 / 图传 / 雷达)
 */

#include "rm_referee27.h"
#include "string.h"
#include "crc_ref.h" /* 复用 modules/referee/crc_ref.c: 官方 CRC8/CRC16 LUT */
#include "bsp_log.h"
#include "cmsis_os.h"

/* 各链路实例(按类型静态分配, 每个类型最多一个) */
static R27Link s_links[R27_LINK_TYPE_CNT];

/* 全局包序号, 所有链路共用(裁判系统不校验序号连续性) */
static uint8_t s_seq;

/* ============================== 链路 -> 回调 ============================== */

static void R27RxCallbackCommon(void);
static void R27RxCallbackVideo(void);
static void R27RxCallbackRadar(void);

/* bsp_usart 的回调无参数, 因此为每种链路准备一个入口, 内部再转发到统一的处理函数 */
static const usart_module_callback s_rx_callbacks[R27_LINK_TYPE_CNT] = {
    R27RxCallbackCommon,
    R27RxCallbackVideo,
    R27RxCallbackRadar,
};

/* 各链路的接收缓冲大小 */
static const uint8_t s_rx_buffer_size[R27_LINK_TYPE_CNT] = {
    R27_COMMON_RX_BUFFER_SIZE,
    R27_VIDEO_RX_BUFFER_SIZE,
    R27_RADAR_RX_BUFFER_SIZE,
};

/* ============================== 内部工具 ============================== */

/**
 * @brief 将收到的数据段按目标结构体大小安全拷贝(短包补零, 长包截断)
 * @note  目标结构体长度已由 referee27_protocol.h 中的 _Static_assert 与手册对齐
 */
static void R27Fill(void *dst, uint16_t dst_size, const uint8_t *src, uint16_t src_len)
{
    uint16_t n = (src_len < dst_size) ? src_len : dst_size;
    memset(dst, 0, dst_size);
    memcpy(dst, src, n);
}

/**
 * @brief 命令码是否属于指定链路
 * @note  图传链路与常规链路共用同一套帧格式, 但命令码互不重叠; 这里做过滤,
 *        避免某条链路上收到脏数据后被写入不相干的结构体。
 */
static uint8_t R27CmdBelongsToLink(R27LinkType_e type, uint16_t cmd)
{
    switch (type)
    {
    case R27_LINK_COMMON:
        switch (cmd)
        {
        case R27_ID_game_state:
        case R27_ID_game_result:
        case R27_ID_game_robot_HP:
        case R27_ID_event_data:
        case R27_ID_referee_warning:
        case R27_ID_dart_info:
        case R27_ID_robot_status:
        case R27_ID_power_heat_data:
        case R27_ID_robot_pos:
        case R27_ID_robot_buff:
        case R27_ID_hurt_data:
        case R27_ID_shoot_data:
        case R27_ID_projectile_allowance:
        case R27_ID_rfid_status:
        case R27_ID_dart_client_cmd:
        case R27_ID_ground_robot_position:
        case R27_ID_radar_mark_data:
        case R27_ID_sentry_info:
        case R27_ID_radar_info:
        case R27_ID_robot_interaction:
        case R27_ID_map_command:
        case R27_ID_map_robot_data:
        case R27_ID_map_data:
        case R27_ID_custom_info:
            return 1;
        default:
            return 0;
        }
    case R27_LINK_VIDEO:
        return (cmd == R27_ID_robot_custom_data || cmd == R27_ID_custom_terminal_cmd) ? 1 : 0;
    case R27_LINK_RADAR:
        return (cmd >= R27_ID_radar_robot_pos && cmd <= R27_ID_radar_key) ? 1 : 0;
    default:
        return 0;
    }
}

/**
 * @brief 解析一帧 0x0301 机器人交互数据(变长)
 * @note  旧实现固定拷贝 11 字节, 会把 118 字节的交互包截断, 这里按实际长度解析
 */
static void R27ParseInteraction(r27_robot_interaction_t *dst, const uint8_t *data, uint16_t data_len)
{
    memset(dst, 0, sizeof(r27_robot_interaction_t));
    if (data_len < R27_INTERACTION_HEAD_LEN)
        return;

    dst->data_cmd_id = (uint16_t)(data[0] | ((uint16_t)data[1] << 8));
    dst->sender_id = (uint16_t)(data[2] | ((uint16_t)data[3] << 8));
    dst->receiver_id = (uint16_t)(data[4] | ((uint16_t)data[5] << 8));

    dst->data_len = (uint16_t)(data_len - R27_INTERACTION_HEAD_LEN);
    if (dst->data_len > R27_INTERACTION_DATA_MAX)
        dst->data_len = R27_INTERACTION_DATA_MAX;

    memcpy(dst->data, data + R27_INTERACTION_HEAD_LEN, dst->data_len);
}

/* ============================== 帧解析 ============================== */

/**
 * @brief 解析一个完整帧的数据段并写入对应结构体
 * @param frame    帧首地址(指向 SOF)
 * @param data_len 帧头中的 data_length
 */
static void R27Dispatch(R27Link *link, uint8_t *frame, uint16_t data_len)
{
    r27_referee_info_t *info = &link->info;
    r27_radar_link_info_t *radar = &link->radar;
    const uint8_t *data = frame + R27_DATA_OFFSET;

    /* 帧头信息 */
    info->FrameHeader.SOF = frame[R27_SOF];
    info->FrameHeader.DataLength = data_len;
    info->FrameHeader.Seq = frame[R27_SEQ];
    info->FrameHeader.CRC8 = frame[R27_CRC8];
    info->CmdID = (uint16_t)(((uint16_t)frame[R27_CMD_ID_OFFSET + 1] << 8) | frame[R27_CMD_ID_OFFSET]);
    info->last_data_length = data_len;
    info->init_flag = 1;

    /* 拷贝宏: 长度不符说明协议版本可能不一致, 计数以便排查, 但仍按最小长度安全拷贝 */
#define R27_PARSE_INFO(dst)                                         \
    do                                                              \
    {                                                               \
        if (data_len != sizeof(dst))                                \
            info->rx_len_mismatch_cnt++;                            \
        R27Fill(&(dst), sizeof(dst), data, data_len);                \
    } while (0)

#define R27_PARSE_RADAR(dst)                                        \
    do                                                              \
    {                                                               \
        if (data_len != sizeof(dst))                                \
            radar->rx_len_mismatch_cnt++;                           \
        R27Fill(&(dst), sizeof(dst), data, data_len);                \
    } while (0)

    if (!R27CmdBelongsToLink(link->type, info->CmdID))
    {
        info->rx_unknown_cnt++;
        return;
    }

    switch (info->CmdID)
    {
    /* ---- 常规链路 ---- */
    case R27_ID_game_state:
        R27_PARSE_INFO(info->GameState);
        break;
    case R27_ID_game_result:
        R27_PARSE_INFO(info->GameResult);
        break;
    case R27_ID_game_robot_HP:
        R27_PARSE_INFO(info->GameRobotHP);
        break;
    case R27_ID_event_data:
        R27_PARSE_INFO(info->EventData);
        break;
    case R27_ID_referee_warning:
        R27_PARSE_INFO(info->RefereeWarning);
        break;
    case R27_ID_dart_info:
        R27_PARSE_INFO(info->DartInfo);
        break;
    case R27_ID_robot_status:
        R27_PARSE_INFO(info->RobotStatus);
        break;
    case R27_ID_power_heat_data:
        R27_PARSE_INFO(info->PowerHeatData);
        break;
    case R27_ID_robot_pos:
        R27_PARSE_INFO(info->RobotPos);
        break;
    case R27_ID_robot_buff:
        R27_PARSE_INFO(info->Buff);
        break;
    case R27_ID_hurt_data:
        R27_PARSE_INFO(info->HurtData);
        break;
    case R27_ID_shoot_data:
        R27_PARSE_INFO(info->ShootData);
        break;
    case R27_ID_projectile_allowance:
        R27_PARSE_INFO(info->ProjectileAllowance);
        break;
    case R27_ID_rfid_status:
        R27_PARSE_INFO(info->RfidStatus);
        break;
    case R27_ID_dart_client_cmd:
        R27_PARSE_INFO(info->DartClientCmd);
        break;
    case R27_ID_ground_robot_position:
        R27_PARSE_INFO(info->GroundRobotPos);
        break;
    case R27_ID_radar_mark_data:
        R27_PARSE_INFO(info->RadarMarkData);
        break;
    case R27_ID_sentry_info:
        R27_PARSE_INFO(info->SentryInfo);
        break;
    case R27_ID_radar_info:
        R27_PARSE_INFO(info->RadarInfo);
        break;
    case R27_ID_map_command:
        R27_PARSE_INFO(info->MapCommand);
        break;
    case R27_ID_map_robot_data:
        R27_PARSE_INFO(info->MapRobotData);
        break;
    case R27_ID_map_data:
        R27_PARSE_INFO(info->MapData);
        break;
    case R27_ID_custom_info:
        R27_PARSE_INFO(info->CustomInfo);
        break;
    case R27_ID_robot_interaction:
        R27ParseInteraction(&info->Interaction, data, data_len);
        break;

    /* ---- 图传链路 ---- */
    case R27_ID_custom_terminal_cmd: /* 0x0311 自定义终端 -> 机器人 */
        R27_PARSE_INFO(info->TerminalCmd);
        break;
    /* 0x0310 是本机发往自定义终端的数据, 本模块只负责发送, 若收到则忽略 */
    case R27_ID_robot_custom_data:
        break;

    /* ---- 雷达链路 ---- */
    case R27_ID_radar_robot_pos:
        R27_PARSE_RADAR(radar->RobotPos);
        break;
    case R27_ID_radar_robot_hp:
        R27_PARSE_RADAR(radar->RobotHP);
        break;
    case R27_ID_radar_robot_ammo:
        R27_PARSE_RADAR(radar->RobotAmmo);
        break;
    case R27_ID_radar_team_status:
        R27_PARSE_RADAR(radar->TeamStatus);
        break;
    case R27_ID_radar_robot_buff:
        R27_PARSE_RADAR(radar->RobotBuff);
        break;
    case R27_ID_radar_key:
        R27_PARSE_RADAR(radar->Key);
        break;
    default:
        break;
    }

#undef R27_PARSE_INFO
#undef R27_PARSE_RADAR

    info->rx_frame_cnt++;
    if (link->type == R27_LINK_RADAR)
    {
        radar->FrameHeader = info->FrameHeader;
        radar->CmdID = info->CmdID;
        radar->init_flag = 1;
        radar->rx_frame_cnt++;
    }
}

/**
 * @brief 从接收缓冲区解析所有完整帧
 * @note  bsp_usart 在每次 DMA/IDLE 事件后会把收到过的区域清零, 因此可以靠
 *        "下一个字节是否为 0xA5" 判断是否还有下一帧(与旧模块的做法一致),
 *        这里再加上缓冲长度与 CRC 校验, 避免脏数据导致写越界。
 */
static void R27ParseBuffer(R27Link *link, uint8_t *buf)
{
    uint16_t offset = 0;
    const uint16_t buf_size = link->usart->recv_buff_size;

    while ((uint16_t)(offset + R27_LEN_HEADER) <= buf_size && buf[offset] == R27_REFEREE_SOF)
    {
        uint8_t *frame = buf + offset;
        uint16_t data_len;
        uint32_t frame_len;

        /* 帧头 CRC8: 校验前 5 字节(CRC 覆盖前 4 字节) */
        if (!Verify_CRC8_Check_Sum(frame, R27_LEN_HEADER))
        {
            link->info.rx_error_cnt++;
            break;
        }

        data_len = (uint16_t)(frame[R27_DATA_LENGTH] | ((uint16_t)frame[R27_DATA_LENGTH + 1] << 8));
        frame_len = (uint32_t)R27_LEN_HEADER + R27_LEN_CMDID + data_len + R27_LEN_TAIL;

        /* 长度非法或超出本次接收范围: 说明帧不完整, 丢弃剩余数据等待下一包 */
        if (frame_len > buf_size || (uint32_t)offset + frame_len > buf_size)
            break;

        /* 整包 CRC16 */
        if (!Verify_CRC16_Check_Sum(frame, frame_len))
        {
            link->info.rx_error_cnt++;
            break;
        }

        R27Dispatch(link, frame, data_len);
        offset = (uint16_t)(offset + frame_len);
    }
}

/* ============================== 回调 ============================== */

static void R27RxHandler(R27Link *link)
{
    if (link == NULL || link->usart == NULL)
        return;

    if (link->daemon != NULL)
        DaemonReload(link->daemon);

    R27ParseBuffer(link, link->usart->recv_buff);
}

static void R27RxCallbackCommon(void)
{
    R27RxHandler(&s_links[R27_LINK_COMMON]);
}

static void R27RxCallbackVideo(void)
{
    R27RxHandler(&s_links[R27_LINK_VIDEO]);
}

static void R27RxCallbackRadar(void)
{
    R27RxHandler(&s_links[R27_LINK_RADAR]);
}

/* 链路掉线回调: 重新启动串口接收 */
static void R27LostCallback(void *arg)
{
    R27Link *link = (R27Link *)arg;
    if (link == NULL || link->usart == NULL)
        return;

    USARTServiceInit(link->usart);
    LOGWARNING("[rm_referee27] link %d lost", (int)link->type);
}

/* ============================== 对外接口 ============================== */

R27Link *R27RefereeInit(R27LinkType_e type, UART_HandleTypeDef *handle)
{
    if ((uint32_t)type >= (uint32_t)R27_LINK_TYPE_CNT || handle == NULL)
    {
        LOGERROR("[rm_referee27] invalid link type or handle");
        return NULL;
    }

    R27Link *link = &s_links[type];
    if (link->usart != NULL) /* 已注册, 直接返回 */
        return link;

    memset(link, 0, sizeof(R27Link));
    link->type = type;

    USART_Init_Config_s conf = {
        .module_callback = s_rx_callbacks[type],
        .usart_handle = handle,
        .recv_buff_size = s_rx_buffer_size[type],
    };
    link->usart = USARTRegister(&conf);
    if (link->usart == NULL)
    {
        LOGERROR("[rm_referee27] usart register failed");
        return NULL;
    }

    Daemon_Init_Config_s daemon_conf = {
        .callback = R27LostCallback,
        .owner_id = link,
        .reload_count = 30, /* 100Hz 递减, 0.3s 未收到数据判定掉线 */
    };
    link->daemon = DaemonRegister(&daemon_conf);

    link->info.init_flag = 1;

    return link;
}

R27Link *R27RefereeGetLink(R27LinkType_e type)
{
    if ((uint32_t)type >= (uint32_t)R27_LINK_TYPE_CNT)
        return NULL;
    return (s_links[type].usart != NULL) ? &s_links[type] : NULL;
}

r27_referee_info_t *R27RefereeGetInfo(R27LinkType_e type)
{
    R27Link *link = R27RefereeGetLink(type);
    return (link != NULL) ? &link->info : NULL;
}

r27_radar_link_info_t *R27RadarGetInfo(R27LinkType_e type)
{
    R27Link *link = R27RefereeGetLink(type);
    return (link != NULL) ? &link->radar : NULL;
}

uint8_t R27RefereeIsOnline(R27LinkType_e type)
{
    R27Link *link = R27RefereeGetLink(type);
    if (link == NULL || link->daemon == NULL)
        return 0;
    return DaemonIsOnline(link->daemon);
}

uint8_t R27RefereeNextSeq(void)
{
    return s_seq++;
}

/**
 * @brief 等待串口空闲(上一次 DMA 发送完成)后发送一帧
 */
void R27RefereeSend(R27Link *link, uint8_t *buf, uint16_t len)
{
    if (link == NULL || link->usart == NULL || buf == NULL || len == 0)
        return;

    /* 最多等待 20ms, 避免总线异常时死等导致任务阻塞 */
    for (uint16_t guard = 0; guard < 20; guard++)
    {
        if (link->usart->usart_handle->gState == HAL_UART_STATE_READY)
            break;
        osDelay(1);
    }

    USARTSend(link->usart, buf, len, USART_TRANSFER_DMA);
}

/**
 * @brief 发送一帧并节流
 * @note  0x0301 上行频率上限 30Hz 且受接收额度限制, UI 刷新用 10Hz 足够,
 *        因此默认挂起 R27_TX_MIN_INTERVAL_MS, 需要更高频率时自行改宏。
 */
void R27RefereeSendThrottled(R27Link *link, uint8_t *buf, uint16_t len)
{
    R27RefereeSend(link, buf, len);
    osDelay(R27_TX_MIN_INTERVAL_MS);
}
