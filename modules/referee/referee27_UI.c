/**
 * @file referee27_UI.c
 * @brief 裁判系统选手端 UI 绘制 / 0x0301 交互数据发送实现
 */

#include "referee27_UI.h"
#include "string.h"
#include "stdio.h"
#include "crc_ref.h"
#include "bsp_log.h"

/* 0x0301 最大包长: 5(帧头) + 2(命令码) + 6(子内容头) + 112(内容) + 2(CRC16) = 127 字节 */
#define R27_UI_TX_BUF_SIZE (R27_LEN_HEADER + R27_LEN_CMDID + R27_INTERACTION_HEAD_LEN + \
                            R27_INTERACTION_DATA_MAX + R27_LEN_TAIL)

static uint8_t s_tx_buf[R27_UI_TX_BUF_SIZE];

/**
 * @brief 打包并通过常规链路发送一条 0x0301 交互数据
 * @param sub_id      子内容 ID
 * @param sender      发送者 ID(本机器人 ID)
 * @param receiver    接收者 ID(选手端 ID / 队友机器人 ID / 0x8080)
 * @param content     内容数据段
 * @param content_len 内容数据段长度, <= 112
 * @note 内部包含节流(osDelay), 详见 R27RefereeSendThrottled
 */
static void R27InteractionSend(uint16_t sub_id, uint16_t sender, uint16_t receiver,
                               const void *content, uint16_t content_len)
{
    uint16_t data_len = (uint16_t)(R27_INTERACTION_HEAD_LEN + content_len);
    uint16_t total = (uint16_t)(R27_LEN_HEADER + R27_LEN_CMDID + data_len + R27_LEN_TAIL);
    uint8_t *p = s_tx_buf;

    if (content_len > R27_INTERACTION_DATA_MAX || total > R27_UI_TX_BUF_SIZE)
    {
        LOGERROR("[referee27_UI] interaction content too long: %d", (int)content_len);
        return;
    }

    /* 帧头 */
    p[R27_SOF] = R27_REFEREE_SOF;
    p[R27_DATA_LENGTH] = (uint8_t)(data_len & 0xFF);
    p[R27_DATA_LENGTH + 1] = (uint8_t)((data_len >> 8) & 0xFF);
    p[R27_SEQ] = R27RefereeNextSeq();
    p[R27_CRC8] = Get_CRC8_Check_Sum(p, R27_LEN_CRC8, 0xFF);

    /* 命令码(小端) */
    p[R27_CMD_ID_OFFSET] = (uint8_t)(R27_ID_robot_interaction & 0xFF);
    p[R27_CMD_ID_OFFSET + 1] = (uint8_t)((R27_ID_robot_interaction >> 8) & 0xFF);

    /* 子内容头(逐字节写入, 避免非对齐访问) */
    p[R27_DATA_OFFSET + 0] = (uint8_t)(sub_id & 0xFF);
    p[R27_DATA_OFFSET + 1] = (uint8_t)((sub_id >> 8) & 0xFF);
    p[R27_DATA_OFFSET + 2] = (uint8_t)(sender & 0xFF);
    p[R27_DATA_OFFSET + 3] = (uint8_t)((sender >> 8) & 0xFF);
    p[R27_DATA_OFFSET + 4] = (uint8_t)(receiver & 0xFF);
    p[R27_DATA_OFFSET + 5] = (uint8_t)((receiver >> 8) & 0xFF);

    /* 内容数据段 */
    if (content_len != 0 && content != NULL)
        memcpy(p + R27_DATA_OFFSET + R27_INTERACTION_HEAD_LEN, content, content_len);

    /* 整包 CRC16 */
    Append_CRC16_Check_Sum(p, total);

    R27RefereeSendThrottled(R27RefereeGetLink(R27_LINK_COMMON), p, total);
}

/* 图形名填充: 按内存地址增大方向填 3 字节, 不足补 0 */
static void R27UIFillName(uint8_t name[3], char graphname[3])
{
    int i;
    for (i = 0; i < 3 && graphname[i] != '\0'; i++)
        name[2 - i] = (uint8_t)graphname[i];
}

/* ============================== 删除图层 ============================== */

void R27_UIDelete(r27_referee_id_t *_id, uint8_t del_operate, uint8_t del_layer)
{
    r27_interaction_layer_delete_t del;

    if (_id == NULL)
        return;

    del.delete_type = del_operate;
    del.layer = del_layer;

    R27InteractionSend(R27_SubID_UI_LayerDelete, _id->Robot_ID, _id->Client_ID, &del, sizeof(del));
}

/* ============================== 图形绘制 ============================== */

void R27_UILineDraw(r27_interaction_figure_t *graph, char graphname[3], uint32_t Graph_Operate,
                    uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width,
                    uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y)
{
    R27UIFillName(graph->figure_name, graphname);

    graph->operate_tpye = Graph_Operate;
    graph->figure_tpye = R27_UI_Graph_Line;
    graph->layer = Graph_Layer;
    graph->color = Graph_Color;

    graph->details_a = 0;
    graph->details_b = 0;
    graph->width = Graph_Width;
    graph->start_x = Start_x;
    graph->start_y = Start_y;
    graph->details_c = 0;
    graph->details_d = End_x; /* 终点 x */
    graph->details_e = End_y; /* 终点 y */
}

void R27_UIRectangleDraw(r27_interaction_figure_t *graph, char graphname[3], uint32_t Graph_Operate,
                         uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width,
                         uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y)
{
    R27UIFillName(graph->figure_name, graphname);

    graph->operate_tpye = Graph_Operate;
    graph->figure_tpye = R27_UI_Graph_Rectangle;
    graph->layer = Graph_Layer;
    graph->color = Graph_Color;

    graph->details_a = 0;
    graph->details_b = 0;
    graph->width = Graph_Width;
    graph->start_x = Start_x;
    graph->start_y = Start_y;
    graph->details_c = 0;
    graph->details_d = End_x; /* 对角顶点 x */
    graph->details_e = End_y; /* 对角顶点 y */
}

void R27_UICircleDraw(r27_interaction_figure_t *graph, char graphname[3], uint32_t Graph_Operate,
                      uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width,
                      uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius)
{
    R27UIFillName(graph->figure_name, graphname);

    graph->operate_tpye = Graph_Operate;
    graph->figure_tpye = R27_UI_Graph_Circle;
    graph->layer = Graph_Layer;
    graph->color = Graph_Color;

    graph->details_a = 0;
    graph->details_b = 0;
    graph->width = Graph_Width;
    graph->start_x = Start_x;
    graph->start_y = Start_y;
    graph->details_c = Graph_Radius; /* 半径 */
    graph->details_d = 0;
    graph->details_e = 0;
}

void R27_UIOvalDraw(r27_interaction_figure_t *graph, char graphname[3], uint32_t Graph_Operate,
                    uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width,
                    uint32_t Start_x, uint32_t Start_y, uint32_t end_x, uint32_t end_y)
{
    R27UIFillName(graph->figure_name, graphname);

    graph->operate_tpye = Graph_Operate;
    graph->figure_tpye = R27_UI_Graph_Ellipse;
    graph->layer = Graph_Layer;
    graph->color = Graph_Color;

    graph->details_a = 0;
    graph->details_b = 0;
    graph->width = Graph_Width;
    graph->start_x = Start_x;
    graph->start_y = Start_y;
    graph->details_c = 0;
    graph->details_d = end_x; /* x 半轴长度 */
    graph->details_e = end_y; /* y 半轴长度 */
}

void R27_UIArcDraw(r27_interaction_figure_t *graph, char graphname[3], uint32_t Graph_Operate,
                   uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_StartAngle,
                   uint32_t Graph_EndAngle, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y,
                   uint32_t end_x, uint32_t end_y)
{
    R27UIFillName(graph->figure_name, graphname);

    graph->operate_tpye = Graph_Operate;
    graph->figure_tpye = R27_UI_Graph_Arc;
    graph->layer = Graph_Layer;
    graph->color = Graph_Color;

    graph->details_a = Graph_StartAngle; /* 起始角度, 0° 为 12 点钟方向, 顺时针 */
    graph->details_b = Graph_EndAngle;   /* 终止角度 */
    graph->width = Graph_Width;
    graph->start_x = Start_x;
    graph->start_y = Start_y;
    graph->details_c = 0;
    graph->details_d = end_x; /* x 半轴长度 */
    graph->details_e = end_y; /* y 半轴长度 */
}

void R27_UIFloatDraw(r27_interaction_figure_t *graph, char graphname[3], uint32_t Graph_Operate,
                     uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Size,
                     uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y,
                     int32_t Graph_Float)
{
    uint32_t value = (uint32_t)Graph_Float;

    (void)Graph_Digit; /* 2027 协议中 details_b 对浮点数无作用 */

    R27UIFillName(graph->figure_name, graphname);

    graph->operate_tpye = Graph_Operate;
    graph->figure_tpye = R27_UI_Graph_Float;
    graph->layer = Graph_Layer;
    graph->color = Graph_Color;

    graph->details_a = Graph_Size; /* 字体大小 */
    graph->details_b = 0;
    graph->width = Graph_Width;
    graph->start_x = Start_x;
    graph->start_y = Start_y;

    /* 32 位值拆成 details_c(10bit) / details_d(11bit) / details_e(11bit), 显示值为 value/1000 */
    graph->details_c = value & 0x3FF;
    graph->details_d = (value >> 10) & 0x7FF;
    graph->details_e = (value >> 21) & 0x7FF;
}

void R27_UIIntDraw(r27_interaction_figure_t *graph, char graphname[3], uint32_t Graph_Operate,
                   uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Size,
                   uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, int32_t Graph_Integer)
{
    uint32_t value = (uint32_t)Graph_Integer;

    R27UIFillName(graph->figure_name, graphname);

    graph->operate_tpye = Graph_Operate;
    graph->figure_tpye = R27_UI_Graph_Int;
    graph->layer = Graph_Layer;
    graph->color = Graph_Color;

    graph->details_a = Graph_Size; /* 字体大小 */
    graph->details_b = 0;
    graph->width = Graph_Width;
    graph->start_x = Start_x;
    graph->start_y = Start_y;

    graph->details_c = value & 0x3FF;
    graph->details_d = (value >> 10) & 0x7FF;
    graph->details_e = (value >> 21) & 0x7FF;
}

void R27_UICharDraw(r27_interaction_string_t *graph, char graphname[3], uint32_t Graph_Operate,
                    uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Size,
                    uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, char *fmt, ...)
{
    va_list ap;

    R27UIFillName(graph->figure.figure_name, graphname);

    graph->figure.operate_tpye = Graph_Operate;
    graph->figure.figure_tpye = R27_UI_Graph_Char;
    graph->figure.layer = Graph_Layer;
    graph->figure.color = Graph_Color;

    graph->figure.details_a = Graph_Size; /* 字体大小 */
    graph->figure.details_b = 0;          /* 字符长度, 下方按实际内容填写 */
    graph->figure.width = Graph_Width;
    graph->figure.start_x = Start_x;
    graph->figure.start_y = Start_y;
    graph->figure.details_c = 0;
    graph->figure.details_d = 0;
    graph->figure.details_e = 0;

    memset(graph->data, 0, sizeof(graph->data));

    {
        /* 协议允许 30 字节字符内容(不一定以 '\0' 结尾), 因此先在 tmp 中格式化再按 30 字节截取 */
        char tmp[sizeof(graph->data) + 1] = {0};

        va_start(ap, fmt);
        vsnprintf(tmp, sizeof(tmp), fmt, ap);
        va_end(ap);

        memcpy(graph->data, tmp, sizeof(graph->data));
        graph->figure.details_b = (uint32_t)strlen(tmp); /* 字符长度 */
    }
}

/* ============================== UI 推送 ============================== */

void R27_UIGraphRefresh(r27_referee_id_t *_id, int cnt, ...)
{
    r27_interaction_figure_t figs[7];
    uint16_t sub_id;
    va_list ap;
    int i;

    if (_id == NULL)
        return;

    /* 协议只定义了 1/2/5/7 个图形的子内容 ID */
    switch (cnt)
    {
    case 1:
        sub_id = R27_SubID_UI_Draw1;
        break;
    case 2:
        sub_id = R27_SubID_UI_Draw2;
        break;
    case 5:
        sub_id = R27_SubID_UI_Draw5;
        break;
    case 7:
        sub_id = R27_SubID_UI_Draw7;
        break;
    default:
        LOGERROR("[referee27_UI] graph refresh cnt must be 1/2/5/7, got %d", cnt);
        return;
    }

    va_start(ap, cnt);
    for (i = 0; i < cnt; i++)
        figs[i] = va_arg(ap, r27_interaction_figure_t); /* 注意: 结构体按值传递需与调用方实参类型一致 */
    va_end(ap);

    R27InteractionSend(sub_id, _id->Robot_ID, _id->Client_ID, figs,
                       (uint16_t)(cnt * sizeof(r27_interaction_figure_t)));
}

void R27_UICharRefresh(r27_referee_id_t *_id, r27_interaction_string_t string_Data)
{
    if (_id == NULL)
        return;

    R27InteractionSend(R27_SubID_UI_DrawChar, _id->Robot_ID, _id->Client_ID,
                       &string_Data, sizeof(r27_interaction_string_t));
}

/* ============================== 其它子内容 ============================== */

void R27_CommBetweenRobotSend(r27_referee_id_t *_id, uint8_t *data, uint16_t data_len)
{
    if (_id == NULL || data == NULL || data_len == 0)
        return;
    if (data_len > R27_INTERACTION_DATA_MAX)
        data_len = R27_INTERACTION_DATA_MAX;

    R27InteractionSend(R27_SubID_RobotComm, _id->Robot_ID, _id->Receiver_Robot_ID, data, data_len);
}

void R27_SentryCmdSend(r27_referee_id_t *_id, r27_sentry_cmd_t *cmd)
{
    if (_id == NULL || cmd == NULL)
        return;

    /* 自主决策指令的接收者为裁判系统服务器 */
    R27InteractionSend(R27_SubID_SentryCmd, _id->Robot_ID, R27_SERVER_CLIENT_ID, cmd, sizeof(r27_sentry_cmd_t));
}

void R27_RadarCmdSend(r27_referee_id_t *_id, r27_radar_cmd_t *cmd)
{
    if (_id == NULL || cmd == NULL)
        return;

    R27InteractionSend(R27_SubID_RadarCmd, _id->Robot_ID, R27_SERVER_CLIENT_ID, cmd, sizeof(r27_radar_cmd_t));
}
