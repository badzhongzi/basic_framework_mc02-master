# refree_27 —— 裁判系统 2027 协议模块

对接手册：**RoboMaster 2027 机甲大师高校系列赛通信协议 V1.0.0（20261029）**

本目录是一个**与 `modules/referee`（2022/2023 协议）完全并存的独立实现**：所有标识符都带
`R27_` / `r27_` 前缀，两套代码可以同时编译、逐步迁移，改动不会影响旧模块和现有应用。

---

## 1. 文件职责

| 文件 | 职责 |
|---|---|
| `referee27_protocol.h` | 帧格式、命令码表（0x0001~0x0308）、0x0301 交互数据、UI 图形结构体、ID 编号（附录二）。全部结构体带 `_Static_assert` 长度/偏移校验 |
| `radar27_protocol.h` | **雷达无线链路**（0x0A01~0x0A06）数据定义、位定义、姿态/状态枚举 |
| `rm_referee27.h/.c` | 链路抽象：`R27Link`（常规 / 图传 / 雷达）、初始化、多帧解析、命令码按链路过滤、收发接口、掉线守护 |
| `referee27_UI.h/.c` | 0x0301 发送侧：UI 绘制（删除图层 / 1/2/5/7 图形 / 字符）、机器人间通信、哨兵（0x0120）与雷达（0x0121）自主决策指令 |
| `referee27_task.h/.c` | UI 刷新任务（模式变化检测 + 静态/动态图元），移植自旧 `referee_task.c` |

CRC 直接复用 `modules/referee/crc_ref.c`（已逐字节比对，与 2027 手册附录一**完全相同**），不重复实现。

---

## 2. 与旧协议（2022/2023）的关键差异

### 2.1 不变的部分

- 帧格式：`0xA5 | data_length(2) | seq | CRC8 | cmd_id(2) | data | CRC16`，CRC8/CRC16 表与算法一致；
- 波特率：常规链路 115200，图传链路 921600；
- 0x0301 子内容 ID 与长度（0x0100 删除、0x0101/0x0102/0x0103/0x0104 图形 15/30/75/105、0x0110 字符 45）；
- 图形结构体 15 字节的**位布局**：只把字段改名（`start_angle→details_a`、`end_angle→details_b`、`radius→details_c`、`end_x→details_d`、`end_y→details_e`），二进制兼容；
- 选手端 ID = `0x0100 + robot_id`（红 1→0x0101，蓝 106→0x016A）；
- 机器人间自定义通信子 ID 区间 `0x0200~0x02FF`，内容数据段最大 112 字节。

### 2.2 命令码变化

| cmd | 旧 | 新 | 说明 |
|---|---|---|---|
| 0x0003 | 32B | **24B** | 只有己方血量 + 伤害差(有符号) + R/B 前哨 + 双方基地 + 双方控制度 |
| 0x0101 | 4B | **8B** | `event_data` + `event_data_2`，位定义大幅扩充（含 RMUL 控制区、对方飞镖命中） |
| 0x0102 / 0x0103 | 4B | **删除** | 补给站动作/预约子弹 |
| 0x0104 / 0x0105 | — | **新增** | 裁判警告(3B) / 飞镖数据(3B) |
| 0x0201 | 13B | **17B** | 新增 `float bullet_speed_limit` |
| 0x0202 | 16B | **14B** | **删除** 底盘电压/电流/功率，只保留 `buffer_energy` + 17mm/42mm 热量 |
| 0x0203 | 16B | **12B** | 去掉 `z`，`yaw`→`angle`（度，正北 0°） |
| 0x0204 | 6B | **7B** | `cooling_buff` u8→**u16**，字段顺序变化 |
| 0x0205 | 2B | **删除** | 空中机器人能量 |
| 0x0206 | 1B | **7B** | 新增命中序号 + 命中坐标(mm) |
| 0x0207 | 7B | 7B | 布局不变，字段改名 |
| 0x0208~0x020E | — | **新增** | 允许发弹量/金币、RFID、飞镖选手端、地面机器人位置、雷达标记、哨兵/雷达决策同步 |
| 0x0301 | 118B | 118B | 上行频率 10Hz→**30Hz**；新增 0x0120 哨兵指令(4B)、0x0121 雷达指令(13B) |
| 0x0303/0x0305/0x0307/0x0308 | — | **新增** | 小地图下发/雷达数据/路径/自定义消息 |
| 0x0310 / 0x0311 | — | **新增** | 图传链路：机器人→自定义终端 300B@50Hz / 终端→机器人 60B@75Hz |
| 0x0A01~0x0A06 | — | **新增** | 雷达无线链路（见第 5 节） |

### 2.3 ID 编号变化（附录二）

- 机器人：`1 重装 / 2 保留 / 3,4,5 步兵 / 6 空中 / 7 哨兵 / 8 飞镖 / 9 雷达 / 10 基地 / 11 前哨`
  （工程机器人已取消，蓝方 = 红方 +100）；
- 选手端：红 `0x0101~0x0106`，蓝 `0x0165~0x016A`，服务器 **`0x8080`**。

---

## 3. 快速上手

### 3.1 常规链路（替代旧的 `UITaskInit`）

```c
r27_interactive_info_t ui_data;                 /* 由上层填写模式/功率等 */

// chassis 初始化中:
r27_referee_info_t *ref =
        R27UITaskInit(&huart6, &ui_data);       /* 内部 = R27RefereeInit(R27_LINK_COMMON, &huart6) */
if (ref == NULL) { /* 初始化失败 */ }

// UI 任务中:
R27MyUIInit();                                  /* 会等待首帧 0x0201 拿到 robot_id */
for (;;) { R27UITask(); osDelay(1); }           /* 10Hz 节流由 R27_TX_MIN_INTERVAL_MS 控制 */
```

读取数据（成员名与手册一致）：

```c
uint16_t limit   = ref->RobotStatus.chassis_power_limit;
float    max_v   = ref->RobotStatus.bullet_speed_limit;
uint16_t buffer  = ref->PowerHeatData.buffer_energy;
uint16_t heat17  = ref->PowerHeatData.shooter_17mm_barrel_heat;
uint16_t ammo    = ref->ProjectileAllowance.projectile_allowance_17mm;
int16_t  dmg_diff= ref->GameRobotHP.damage_difference;
uint32_t ev      = ref->EventData.event_data;
uint32_t rune    = R27_EVENT_GET(ev, R27_EVENT_SMALL_RUNE);   /* 位域解码辅助宏 */
```

### 3.2 图传链路（第二路串口，921600）

```c
R27Link *video = R27RefereeInit(R27_LINK_VIDEO, &huart7);
/* 发送 0x0310: 300 字节自定义数据, 上限 50Hz, 需自己做频率控制 */
static uint8_t payload[R27_LEN_robot_custom_data];
R27RefereeSend(video, frame, total_len);        /* 此处 frame 需按帧格式自行打包 */
/* 接收 0x0311: 60 字节, 解析结果在 info->TerminalCmd.data[] */
```

### 3.3 雷达链路

```c
R27Link *radar = R27RefereeInit(R27_LINK_RADAR, &huart8);   /* 雷达接收机串口转发 */
r27_radar_link_info_t *rd = R27RadarGetInfo(R27_LINK_RADAR);
uint16_t enemy_sentry_x = rd->RobotPos.opponent_sentry_position_x;   /* cm */
uint8_t  sentry_posture = rd->RobotBuff.sentry_posture;
uint8_t  enemy_key[6]; memcpy(enemy_key, rd->Key.key, 6);
```

---

## 4. UI 与 0x0301 发送

```c
r27_referee_id_t *id = R27GetRefereeId();

R27_UICharDraw(&str, "ss0", R27_UI_Graph_ADD, 8, R27_UI_Color_Main, 15, 2, 150, 750, "chassis:");
R27_UICharRefresh(id, str);

r27_interaction_figure_t g;
R27_UILineDraw(&g, "sl0", R27_UI_Graph_ADD, 7, R27_UI_Color_White, 3, 710, 540, 1210, 540);
R27_UIGraphRefresh(id, 1, g);                   /* 只支持 1/2/5/7 个 */

/* 机器人间通信(子内容 0x0200~0x02FF) */
id->Receiver_Robot_ID = R27_RobotID_RStandard_4;
R27_CommBetweenRobotSend(id, buf, len);         /* len <= 112 */

/* 哨兵/雷达自主决策指令(接收者固定 0x8080) */
r27_sentry_cmd_t sc = {0}; sc.sentry_cmd = 3u << R27_SENTRY_CMD_POSTURE_SHIFT;
R27_SentryCmdSend(id, &sc);
r27_radar_cmd_t rc = {0}; rc.radar_cmd = 1;
R27_RadarCmdSend(id, &rc);
```

> `R27_UIFloatDraw` 的 `Graph_Digit` 形参仅为兼容旧调用保留（2027 手册标注 `details_b` 对浮点数"无作用"）。

---

## 5. 雷达链路：为什么只单开一个头文件

| 问题 | 结论 |
|---|---|
| 需要单独的 `.c` 收发实现吗？ | **不需要**。雷达链路与常规链路共用同一套帧格式与 CRC，`rm_referee27.c` 的解析器已按 `R27_LINK_RADAR` 分流，数据落到 `rd->RobotPos / RobotHP / RobotAmmo / TeamStatus / RobotBuff / Key` |
| 需要单独的头文件吗？ | **需要**。6 条命令码 + 位定义 + 姿态/状态枚举共约 200 行，塞进主协议头会让常规链路部分难以阅读，因此独立成 `radar27_protocol.h` |
| 什么时候才需要单开模块？ | 只有当雷达接收机改走**以太网/UDP**时。那时帧格式不再是裁判系统串口帧，应新建 `modules/radar_link`（或放在上位机/视觉端），不要塞进本模块 |
| 现有实现限流点 | 单包最大 56 字节（0x0A05），`128` 字节接收缓冲足够；6 条命令按 10Hz 合计约 1.1KB/s |

注意区分两类"雷达数据"：
- **雷达无线链路**（0x0A01~0x0A06）：信号发射源 → 雷达机器人，单向广播，只有雷达能收到；
- **常规链路**里属于雷达的命令：0x020C（收标记进度）、0x020E（收雷达自身状态）、0x0121（发自主决策）、0x0305（把双方坐标发给选手端小地图）。

---

## 6. 已知限制 / 后续 TODO

1. **bsp 接收缓冲**：`bsp_usart.h` 的 `USART_RXBUFF_LIMIT = 256`、`recv_buff_size` 为 `uint8_t`。
   - 常规/图传/雷达链路**接收**都够用（最大 127 / 69 / 56 字节）；
   - 但若要**接收** 0x0310（300B，总包 309B）就必须把 `recv_buff_size` 改成 `uint16_t` 并放大 `USART_RXBUFF_LIMIT`。机器人侧只发不收 0x0310，故本次未改 bsp。
2. **图传 0x0310 打包**：目前只提供通用的 `R27RefereeSend()`，如需 50Hz 稳定发送，建议上层加双缓冲 + DMA 队列（`USARTSend` 是单次 DMA，连续调用需等 `gState == READY`，本模块已做 20ms 超时保护）。
3. **带宽**：0x0301 上行受额度限制（重装/步兵 3720B/s，哨兵/雷达 5120B/s × λ），UI 刷新目前节流 100ms/包；堆叠多包时请先核算。
4. **未实现但已定义好结构**：0x0303（小地图下发，含服务器 5 次补发 + 1Hz 定频）、0x0305/0x0307/0x0308（发给选手端）、0x0310 内容格式、自定义终端 Protobuf/MQTT（走 PC 端，与 MCU 无关）。
5. **应用层迁移**：`application/chassis/chassis.c` 仍使用旧模块（`referee_data->GameRobotState.chassis_power_limit`）。
   迁移时只需把 `UITaskInit(&huart6, &ui_data)` 换成 `R27UITaskInit()`，字段改为
   `ref->RobotStatus.chassis_power_limit`；`referee_data->PowerHeatData.chassis_power` 在新协议中**已不存在**，功率控制只能用 `buffer_energy` + 自测功率。

---

## 7. 验证记录

1. `_Static_assert` 覆盖 26 个命令码结构体长度 + 9 个关键字段偏移（含 0x020D 的 8 字节非对齐 `uint64`），
   编译期即可发现"手册更新但代码未改"的问题；
2. host 侧用 x86 gcc 编译协议头 + `crc_ref.c`，手工构造 0x0201 帧：
   `CRC8/CRC16 校验通过`、`robot_id/power_limit/bullet_speed_limit/power_management` 回读正确；
3. `make -j` 全量编译通过，新模块无警告（`-Wall`）；因暂无调用者，链接阶段被 `--gc-sections` 回收，不占用固件空间。
