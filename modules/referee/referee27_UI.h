/**
 * @file referee27_UI.h
 * @brief 裁判系统选手端 UI 绘制 / 0x0301 交互数据发送接口
 *
 * @note 本文件负责所有经 0x0301 发送的内容:
 *       - 子内容 0x0100~0x0110: 选手端 UI(删除图层 / 绘制图形 / 绘制字符)
 *       - 子内容 0x0120/0x0121: 哨兵/雷达自主决策指令
 *       - 子内容 0x0200~0x02FF: 机器人之间通信
 *
 * @note 与旧模块 referee_UI 的差异:
 *       1. 图形结构体字段改为手册 2027 的 details_a~e 命名(位布局与旧协议一致, 二进制兼容);
 *       2. 浮点数不再使用 details_b(手册标注"无作用"), 但为兼容旧调用保留 Graph_Digit 形参;
 *       3. 发送缓冲区按 0x0301 的最大包长(127 字节)声明, 不再用 512 字节的固定缓存。
 */

#ifndef REFEREE27_UI_H
#define REFEREE27_UI_H

#include "stdarg.h"
#include "stdint.h"
#include "referee27_protocol.h"
#include "rm_referee27.h"

/* UI 发送帧结构(用于自查/调试, 实际打包在 referee27_UI.c 内部完成) */
#pragma pack(1)
typedef struct
{
    r27_frame_header_t FrameHeader;
    uint16_t CmdID;
    r27_interaction_header_t datahead;
    uint8_t Delete_Operate; /* 删除操作 */
    uint8_t Layer;          /* 图层 */
    uint16_t frametail;     /* CRC16 */
} r27_ui_delete_t;

typedef struct
{
    r27_frame_header_t FrameHeader;
    uint16_t CmdID;
    r27_interaction_header_t datahead;
    uint16_t frametail;
} r27_ui_graph_refresh_t;

typedef struct
{
    r27_frame_header_t FrameHeader;
    uint16_t CmdID;
    r27_interaction_header_t datahead;
    r27_interaction_string_t String_Data;
    uint16_t frametail;
} r27_ui_char_refresh_t;
#pragma pack()

/* ============================== UI 绘制 ============================== */

/**
 * @brief 删除图层
 * @param del_operate R27_UI_Del_NoOperate / R27_UI_Del_Layer / R27_UI_Del_ALL
 * @param del_layer   图层数 0~9(删除所有时忽略)
 */
void R27_UIDelete(r27_referee_id_t *_id, uint8_t del_operate, uint8_t del_layer);

void R27_UILineDraw(r27_interaction_figure_t *graph, char graphname[3], uint32_t Graph_Operate,
                    uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width,
                    uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y);

void R27_UIRectangleDraw(r27_interaction_figure_t *graph, char graphname[3], uint32_t Graph_Operate,
                         uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width,
                         uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y);

void R27_UICircleDraw(r27_interaction_figure_t *graph, char graphname[3], uint32_t Graph_Operate,
                      uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width,
                      uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius);

void R27_UIOvalDraw(r27_interaction_figure_t *graph, char graphname[3], uint32_t Graph_Operate,
                    uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width,
                    uint32_t Start_x, uint32_t Start_y, uint32_t end_x, uint32_t end_y);

void R27_UIArcDraw(r27_interaction_figure_t *graph, char graphname[3], uint32_t Graph_Operate,
                   uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_StartAngle,
                   uint32_t Graph_EndAngle, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y,
                   uint32_t end_x, uint32_t end_y);

/**
 * @brief 浮点数图形
 * @param Graph_Float 实际显示值 * 1000 的 32 位整数(例: 欲显示 1.234 -> 传入 1234)
 * @param Graph_Digit 已废弃(2027 协议中 details_b 无作用), 仅为兼容旧调用保留
 */
void R27_UIFloatDraw(r27_interaction_figure_t *graph, char graphname[3], uint32_t Graph_Operate,
                     uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Size,
                     uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y,
                     int32_t Graph_Float);

void R27_UIIntDraw(r27_interaction_figure_t *graph, char graphname[3], uint32_t Graph_Operate,
                   uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Size,
                   uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, int32_t Graph_Integer);

/**
 * @brief 字符图形(类 printf)
 * @param Graph_Size 字号, 建议 字号:线宽 = 10:1
 */
void R27_UICharDraw(r27_interaction_string_t *graph, char graphname[3], uint32_t Graph_Operate,
                    uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Size,
                    uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, char *fmt, ...);

/**
 * @brief 推送图形修改
 * @param cnt 图形个数, 只支持 1 / 2 / 5 / 7(协议规定的子内容 ID 只有这四种)
 */
void R27_UIGraphRefresh(r27_referee_id_t *_id, int cnt, ...);

/**
 * @brief 推送字符修改
 */
void R27_UICharRefresh(r27_referee_id_t *_id, r27_interaction_string_t string_Data);

/* ============================== 0x0301 其它子内容 ============================== */

/**
 * @brief 机器人之间通信(子内容 0x0200~0x02FF)
 * @param data 数据段, data_len <= 112
 */
void R27_CommBetweenRobotSend(r27_referee_id_t *_id, uint8_t *data, uint16_t data_len);

/**
 * @brief 哨兵自主决策指令(子内容 0x0120), 接收者为裁判系统服务器(0x8080)
 */
void R27_SentryCmdSend(r27_referee_id_t *_id, r27_sentry_cmd_t *cmd);

/**
 * @brief 雷达自主决策指令(子内容 0x0121), 接收者为裁判系统服务器(0x8080)
 */
void R27_RadarCmdSend(r27_referee_id_t *_id, r27_radar_cmd_t *cmd);

#endif /* REFEREE27_UI_H */
