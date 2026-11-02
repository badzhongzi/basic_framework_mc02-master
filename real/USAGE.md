# real/ 使用说明

real/
├── mpc_config.py          ← 唯一参数入口（改参数只改这里）
├── dyn_model.py           ← 等效惯量、惯量比、观测器、力矩边界、饱和
├── cmpc.py                ← P7 求解器：消元 + 定迭代 ADMM（仿真和生成器共用）
├── gen_code.py            ← 生成 C 代码的脚本
├── sim.py                 ← 闭环仿真（验证精度/约束/恢复时间）
├── sim_core.py            ← 仿真内核
├── plot_npz.py            ← 把 sim_result.npz 画成离线 HTML
├── sweep_rt.py            ← 旧 OSQP 求解器的参数扫描（只对 SOLVER="osqp" 有意义）
├── USAGE.md               ← 本文件
├── ctest/                 ← C 侧 float32 数值回归
│   ├── build.sh           ← 编译+运行三个回归
│   ├── test_qp.c          ← 求解器回归：mpc_cs_step() vs Python
│   ├── test_rnea.c        ← RNEA 回归：CasADi float32 vs pinocchio float64
│   ├── test_loop.c        ← 整个 tick 循环回归
│   ├── dump_qp.py / dump_rnea.py / dump_loop.py   ← 导出用例
└── generated/             ← 产物（绝对不要手改）
    ├── mpc_6axis.h/.c     ← MPC 全部在线逻辑：观测器、惯量比、边界、ADMM 求解器
    ├── rnea_6axis.h/.c    ← CasADi 生成的 RNEA（前馈力矩）
    ├── crba_6axis.h/.c    ← CasADi 生成的惯量矩阵 M(q)（P3 用）
    └── sources.txt        ← 编译清单

这套脚本做一件事：从 URDF 出发，设计并验证一个「完整 RNEA 前馈 + 6 个独立关节
MPC，每拍全部求解」的控制器，然后生成 STM32H723 能直接编译的 C 代码。
**不依赖 OSQP**：求解器是 `mpc_6axis.c` 里约 100 行的定迭代 ADMM。

控制律是

```text
tau    = sat( tau_ff + Δτ )                    sat = 按 effort 饱和
tau_ff = RNEA(q_测量, dq_测量, ddq_参考) + 摩擦补偿    全自由度，一次算 6 个
Δτ     = ρ(q)·u                                u = 每关节独立 MPC 的解
```

前馈是主项，MPC 只负责补模型误差、抑制扰动、守住约束。

---

## 0. 环境

```text
python  D:/Minforge/envs/robotics/python.exe     pinocchio 4.1.0 / casadi 3.7.2
        （osqp 1.1.3 只在仿真里做旧求解器对照和 ADMM 精度参考时用）
gcc     只有跑 ctest/ 才需要（MinGW 或任意 C11 编译器）
```

下面所有命令都在 `real/` 目录下执行。Windows 控制台是 GBK，脚本输出里有
`ŵ` 这类字符，管道或重定向时先 `export PYTHONIOENCODING=utf-8`。

---

## 1. 文件职责

| 文件 | 干什么 | 要不要手改 |
|---|---|---|
| `mpc_config.py` | **唯一的参数入口**，仿真和代码生成共用 | 改参数只改这里 |
| `dyn_model.py` | 等效惯量、惯量比、观测器设计、力矩边界、饱和 | 不用 |
| `cmpc.py` | 求解器离线设计（矩阵、ρ、K⁻¹）和在线求解，C 侧逐行对应 | 不用 |
| `sim_core.py` | 闭环仿真内核 + 统计定义 | 不用 |
| `sim.py` | 跑一次完整仿真，打印统计、存 npz | 不用 |
| `gen_code.py` | 生成 `generated/` 下的全部 C 代码 | 不用 |
| `ctest/` | C 与 Python 的 float32 数值回归 | 不用 |
| `generated/` | 产物，每次生成重建 | **绝对不要** |

---

## 2. 典型流程

```bash
# 1) 改参数：编辑 mpc_config.py

# 2) 仿真验证：看精度、约束、恢复时间（约 80 s）
D:/Minforge/envs/robotics/python.exe -u sim.py

# 3) 生成 C 代码
D:/Minforge/envs/robotics/python.exe -u gen_code.py

# 4) 验证生成的 C 和 Python 数值一致（dump 共约 4 分钟）
cd ctest
D:/Minforge/envs/robotics/python.exe -u dump_qp.py
D:/Minforge/envs/robotics/python.exe -u dump_rnea.py
D:/Minforge/envs/robotics/python.exe -u dump_loop.py
sh build.sh
```

第 2 步和第 3 步**必须成对重跑**。原因见第 4 节。

---
## 3. 架构一页纸

**误差与扰动模型。** 被控对象和控制器用的不是同一套东西，这样 MPC 才有事情做：

| 来源 | 配置项 | 当前值 |
|---|---|---|
| 编码器量化 + 噪声 | `NOISE_Q_LSB` / `NOISE_DQ_LSB` / `FEEDBACK_NOISE` | 按上位机设定的 P/V_MAX 算的 1 LSB |
| 关节摩擦（库伦+粘性） | `FRICTION_COULOMB` / `FRICTION_VISCOUS` | 暂定值，等台架实测 |
| 末端负载突变 | `PAYLOAD_T` / `PAYLOAD_MASS` / `PAYLOAD_COM` | t=12 s 加 0.4 kg，末端系 +Z 0.08 m |
| 碰撞冲击 | `IMPACT_T` / `IMPACT_TAU` / `IMPACT_DURATION` | t=18 s，50 ms 力矩脉冲 |
| 对象质量偏差 | `PLANT_MASS_SCALE` | 1.05 |
| 执行 / 反馈延迟 | `CTRL_DELAY_TICKS` / `FEEDBACK_DELAY_TICKS` | 各 1 拍，环路共 4 ms |

被控对象每个控制周期积分 `PLANT_SUBSTEPS` 步（自动算，现在是 13 步），否则零速
附近的库伦摩擦在手腕上数值失稳。精度统计一律用 `e_true`（真实误差）。

**控制器的改进项。** 都可以单独关掉做对照：

| | 做什么 | 配置 |
|---|---|---|
| P1 扰动观测 | 每关节 3 状态 Kalman 观测器 [e, de, w]，ŵ 写进 MPC 预测，即积分作用 | `ESTIMATOR`、`USE_DIST_COMP`、`OBS_*` |
| P2 延迟补偿 | 观测器在测量时刻更新，用已发出的 u 前推到力矩生效时刻；前馈也用生效时刻的参考 | `DELAY_COMP`、`EST_*_DELAY_TICKS` |
| P3 惯量缩放 | Δτ = ρ(q)·u，ρ = M_eff(q)/M_design。QP 矩阵不变，修正量随构型缩放 | `MPC_FORM = "ddq"` |
| P4 取消错峰 | 每 2 ms 解全部 6 个关节，预测步长 = 2 ms（以前每关节 6 ms 才更新一次） | `SOLVER = "cs_admm"` |
| P5 状态约束 | 关节位置、速度限位作为软约束，沿时域检查；总力矩上限沿时域按预测前馈变化 | `JOINT_*`、`SOFT_*_WEIGHT` |
| P7 新求解器 | 消元 + 分块 + 定迭代 ADMM，所有矩阵离线算好，耗时是常数 | `MPC_BLOCKS`、`CS_*`、`ADMM_*` |

`SOLVER = "osqp"` 保留旧的 OSQP + 错峰，只在仿真里做对照；`gen_code.py` 只生成新的。

**P1/P2/P3 的对照**（标准场景、真实延迟 4 ms；六个关节取最大）：

| 指标 | 改之前 | P1+P2+P3 |
|---|---|---|
| 静止段静差 | 0.00167 rad | 0.00022 rad |
| 负载段静差 | 0.00401 rad | 0.00023 rad |
| 冲击后恢复时间 | 0.122 s | 0.076 s |
| 真实延迟 8 ms 时轨迹误差 | 0.04621 rad | 0.00883 rad |

**P4/P5/P7 的对照**（同一场景，旧 = OSQP + 错峰，新 = 定迭代 ADMM + 每拍全解）：

| 指标 | 旧 | 新 |
|---|---|---|
| 轨迹段 max\|e\| | 0.00380 rad | 0.00351 rad |
| 静止段静差 | 0.00022 rad | 0.00024 rad |
| 负载段静差 | 0.00023 rad | 0.00016 rad |
| 冲击 max\|e\| | 0.03493 rad | 0.03476 rad |
| 冲击后恢复时间 | 0.076 s | 0.080 s |
| 静止时速度抖动 | 0.0241 rad/s | 0.0160 rad/s |
| 真实延迟 8 ms 时轨迹误差 | 0.00883 rad | 0.00376 rad |
| 迭代次数 | 不固定，最坏 35 | 固定 25 |

约束不起作用时新旧控制器的线性增益是对齐过的（偏差 < 1.6%，见 `CS_R_WEIGHT`
的注释），所以标准场景下两者差不多是预期结果。新版的收益在三处：
延迟裕度（每拍重解，8 ms 延迟时误差小一半多）、确定的耗时、约束。

**P5 约束起作用的场景**（`ctest/dump_qp.py` 里也用这两个场景做 C 回归）：

| 场景 | 新, 约束开 | 新, 约束关 | 旧 |
|---|---|---|---|
| 速度：J0 参考峰值 4.69 rad/s，限位 3.0 | max\|dq\| **3.010** | 4.69 | 4.69 |
| 位置：J1 目标 3.30，约束线 3.12 | max q **3.1226** | 3.3008 | 3.3008 |

速度约束起作用时轨迹段跟踪误差会到 0.48 rad（参考本身不可行，MPC 选择限速），
参考慢下来之后恢复。冲击把关节往限位外推时约束帮不上忙：Δτ 已经顶在
`DTAU_MAX` 上，约束开和关结果完全相同，瓶颈是修正权限而不是约束。

**求解器结构（P7）。** 每个关节一个独立的小 QP：

```text
状态    w = [e, de]，e = q_ref - q（误差模型，参考预览在代价里被消掉）
模型    w⁺ = A w + B u + Cw ŵ           精确零阶保持，T = 2 ms
时域    30 拍 = 60 ms，分 10 块 MPC_BLOCKS = [1,1,1,2,2,3,4,4,6,6]，块内 u 不变
消元    w_k = Φ_k w0 + Γ_k u + Ψ_k ŵ    QP 只剩 10 个变量 u
代价    ½ uᵀHu + fᵀu，H 常数，f = Gw·w0 + Gc·ŵ
约束    u 的箱约束（硬）：|Δτ| <= DTAU_MAX 且 |tau_ff(k) + Δτ| <= effort，
                         tau_ff(k) 沿时域从当前值线性插值到时域末尾的预测值
        每块末尾的 q、dq（软，二次罚）：q ∈ [Q_LO, Q_HI]，|dq| <= V_HI
ADMM    K⁻¹ 离线求好；每次迭代 3 个小矩阵乘向量 + 逐元素投影；固定 25 次
        热启动：u、z、y 跨拍保留；ρ 按关节取 0.05·sqrt(λmin·λmax)
```

为什么要软约束：起点已经越界时（冲击、上电在限位外）硬状态约束无解，
软约束保证求解器永远有界。为什么控制量是硬约束：每次迭代都精确投影，
输出 `zu[0]` 一定在箱内，总力矩不可能超限（这是负责人明确要求的）。

定迭代 25 次的精度：对高精度 OSQP 参考解，Δτ 误差 ≤ 0.27% DTAU_MAX
（含约束起作用的工况）。ρ 系数 ≥ 0.3 时收敛明显变差，不要随手调大。

**参考预览在哪里起作用。** 误差模型的代价里，参考的未来走向被消掉了（已经证明过），
所以以前的 `delta_seq` 对代价没有任何影响。现在预览用在约束上：位置、速度限位是
**绝对量**，换成误差的边界要用每个检查点时刻的参考；总力矩上限要用时域末尾的
参考前馈。这就是 C 侧要传 `qr_e/dqr_e` 和 `tau_ff_end` 的原因。

**当前关键量**（改了 `mpc_config.py` 记得对照 `gen_code.py` 的输出更新）：

| 量 | 值 | 来自 |
|---|---|---|
| 模型 | `D:/rm/urdf_urdf/urdf.urdf`，6R | `URDF_PATH` |
| 电机 | J0..J5 = 4340P, 6248P, 6248P, 4310P, 4340P, 4310 | `ARMATURE` 的注释 |
| 转子惯量（折算） | [0.03058, 0.13689, 0.13472, 0.00175, 0.03089, 0.00230] | `ARMATURE` |
| 力矩上限 effort | [28, 48, 48, 12.5, 28, 12.5] Nm | URDF |
| 修正权限 | [8, 15, 12, 3, 3, 1.5] Nm | `DTAU_MAX` |
| 设计惯量 | 工作空间最小 `1/(M⁻¹)_jj` = [0.0368, 0.378, 0.203, 0.00476, 0.0345, 0.00262] | `INERTIA_MODE` |
| 无约束首拍增益 K_pos | [165, 292, 292, 27.2, 158, 14.9] | `cmpc.design()` |
| 无约束首拍增益 K_vel | [6.02, 16.6, 14.1, 0.95, 5.75, 0.53] | 同上 |
| H 条件数 | 9.7 ~ 330 | 同上 |
| 限位 | q ∈ [-3.12, 3.12]（±3.14 扣 0.02），\|dq\| <= 3.0（URDF velocity） | `JOINT_*`、`*_MARGIN` |

为什么惯量用 `1/(M⁻¹)_jj` 而不是 `M_jj`：完整前馈下真实闭环是
`ddq = ddq_ref + M(q)⁻¹Δτ`，单独施加 `Δτ_j` 得到的是 `(M⁻¹)_jj·Δτ_j`。取工作空间
最小值是因为求解器矩阵是编译期常数——低估惯量会过增益发散，高估只是变慢。
P3 再在线乘 ρ(q) 把高估的部分补回来。详见 `dyn_model.py` 的模块注释。

---

## 4. 改参数的规则

**所有参数只在 `mpc_config.py` 里改。** `sim_core.py` 的 `Settings` 会校验键名，
拼错的名字会立刻报错而不是被静默忽略。

**`sim.py` 和 `gen_code.py` 必须成对重跑**，只跑一个会得到一个「仿真验过的参数
和板上跑的参数不是同一套」的系统。耦合点在惯量设计：

```text
Q_GOAL, Q_INIT, INERTIA_MODE, INERTIA_SAMPLES, INERTIA_SEED
    -> dyn_model.design_inertia()  取工作空间内 M_eff 的最小值
    -> cmpc.design()               求解器全部矩阵、钳位上限
```

也就是说，改一个看起来只跟仿真轨迹有关的 `Q_GOAL`，会改变生成的 C 常数。

改完之后至少确认（都在 `sim.py` 的输出里）：

| 看什么 | 期望 | 不对的话 |
|---|---|---|
| `e稳态` / 未恢复数 | 0 / 0 | 有关节没稳住 |
| 总力矩截断次数 `n_tau_saturated` | 0 或很少 | 前馈本身逼近 effort |
| `gen_code.py` 打印的 H 条件数 | 几百以内 | 权重比例失衡，ADMM 25 次不够 |

改了 `CS_Q_*` / `CS_R_WEIGHT` / `MPC_BLOCKS` 之后，要用 `cmpc.reference_solution()`
重新核一次 25 次迭代的精度（做法：仿真里记录 `qp_records`，逐条和参考解比）。

---

## 5. 验证生成的 C：`ctest/`

Python 仿真全程 float64，板上是 float32。这两套代码算的是不是同一件事，必须验，
不能靠「应该一样」。命令见第 2 节第 4 步。`build.sh` 同时也是**生成代码能不能编过**
的检查；初始化式写错（花括号层数、元素个数）这类 gcc 只报 warning 的问题会被拦成错误。

**`test_qp`** —— 单次求解回归，6720 次求解，分三个场景：标准（轨迹段、负载突变、
冲击）、速度限位、位置限位（后两个是 P5 约束起作用的工况）。每条记录带求解前的
热启动状态，两种回放：

- 逐条：每条都拷 Python 的热启动状态，只验这一次求解
- 连续：每段只拷第一条，之后用 C 自己的状态，验 float32 误差会不会沿热启动累积

另外单独比对钳位上限和控制量边界构造。当前结果：钳位、边界一致；逐条和连续的
Δτ 最大偏差都是 0.0018 Nm（0.012% 本关节 DTAU_MAX）；控制量不出箱；总力矩越限 0。

**`test_rnea`** —— 生成的 `rnea_6axis.c`（float32）对 `pin.rnea`（float64）。
判据取 1% 本关节 DTAU_MAX：前馈误差对 MPC 来说和外部扰动无法区分。
gravity / sim 组最大误差 4e-6 Nm，extreme 组（dq 拉到 130 rad/s）相对误差 7.9e-5。

**`test_loop`** —— 整个 tick 循环，4000 步分三段（轨迹运动 0~6 s、负载突变、冲击），
每段起点从 Python 拷观测器和求解器状态。C 自己算前馈、两次参考 RNEA（时域末尾
前馈预测）、惯量比、观测器、6 个关节的求解、装配饱和。当前结果：总力矩最大偏差
6.6e-5 Nm（判据 0.03 Nm），ŵ 相对差 5.4e-5，总力矩越限 0。

---

## 6. C 侧集成

要编译进工程的文件（清单也在 `generated/sources.txt`）：

```text
generated/mpc_6axis.c     观测器、惯量比、边界、定迭代 ADMM、钳位、饱和
generated/rnea_6axis.c    CasADi 展开的 6 自由度 RNEA（前馈）
generated/crba_6axis.c    CasADi 展开的惯量矩阵 M(q)（P3，mpc_6axis.c 依赖它）
include 目录 generated/，链接 libm
```

不再需要 OSQP 的任何文件。用 `arm-none-eabi-gcc 13.3 -O2 -mcpu=cortex-m7
-mfpu=fpv5-d16 -mfloat-abi=hard` 编译：

| 文件 | text（代码+常数表） | bss |
|---|---|---|
| mpc_6axis.o | 14.5 KB（其中常数表约 11 KB） | 1.8 KB |
| rnea_6axis.o | 7.5 KB | 0 |
| crba_6axis.o | 5.0 KB | 0 |

旧版 OSQP 的 6 个 workspace 要约 72 KB RAM。

**需要你自己提供的只有一个东西：轨迹求值函数。** 它要能在任意时刻给出
`(q_ref, dq_ref, ddq_ref)`：过去（`MPC_EST_FB` 拍前，观测器用）、力矩生效时刻
（`MPC_EST_CT` 拍后）、以及之后 `MPC_CS_ENDS[i]` 拍的 10 个检查点和时域末尾
`MPC_CS_NS` 拍。板上的 ins_traj 是可求值计划，满足这一点；时刻超过计划终点时
返回终点（速度、加速度为 0）。

下面这段的**调用顺序**已经被 `ctest/test_loop.c` 逐步验证过（和 Python 仿真
逐拍一致）；轨迹求值、电机收发这些外围部分仍是示意。

```c
#include "mpc_6axis.h"
#include "rnea_6axis.h"

static uint32_t tick;

/* 上电、急停恢复、切换控制模式时调 */
void control_init(void) { mpc_est_reset(); mpc_cs_reset(); }

/* tref = RNEA(参考) + 摩擦补偿(参考速度)，时域末尾前馈预测用 */
static void tref(float t, float out[MPC_NJ])
{
    float q[MPC_NJ], dq[MPC_NJ], ddq[MPC_NJ], fr[MPC_NJ];
    traj_eval(t, q, dq, ddq);
    const casadi_real *arg[3] = { q, dq, ddq };
    casadi_real *res[1] = { out };
    rnea(arg, res, 0, 0, 0);
    mpc_friction_ff(dq, fr);
    for (int j = 0; j < MPC_NJ; ++j) out[j] += fr[j];
}

/* 每 MPC_BASE_DT 调一次。q/dq 是电机反馈换算到关节空间的测量值，tau_out 发给电机。 */
void control_tick(const float q[MPC_NJ], const float dq[MPC_NJ], float tau_out[MPC_NJ])
{
    const float t       = tick * MPC_BASE_DT;
    const float t_meas  = t - MPC_EST_FB * MPC_BASE_DT;   /* 测量对应的时刻 */
    const float t_apply = t + MPC_EST_CT * MPC_BASE_DT;   /* 力矩真正生效的时刻 */

    float qr_m[MPC_NJ], dqr_m[MPC_NJ], qr_a[MPC_NJ], dqr_a[MPC_NJ], ddqr_a[MPC_NJ], tmp[MPC_NJ];
    traj_eval(t_meas, qr_m, dqr_m, tmp);
    traj_eval(t_apply, qr_a, dqr_a, ddqr_a);

    /* 1. 前馈 = RNEA(测量 q, 测量 dq, 生效时刻的参考 ddq) + 摩擦补偿 */
    float tau_ff[MPC_NJ], fric[MPC_NJ];
    const casadi_real *arg[3] = { q, dq, ddqr_a };
    casadi_real *res[1] = { tau_ff };
    rnea(arg, res, 0, 0, 0);
    mpc_friction_ff(dq, fric);
    for (int j = 0; j < MPC_NJ; ++j) tau_ff[j] += fric[j];

    /* 2. 时域末尾的前馈预测（P5 总力矩约束沿时域变化） */
    float t0[MPC_NJ], t1[MPC_NJ], tau_ff_end[MPC_NJ];
    tref(t_apply, t0);
    tref(t_apply + MPC_CS_NS * MPC_BASE_DT, t1);
    for (int j = 0; j < MPC_NJ; ++j) tau_ff_end[j] = tau_ff[j] + (t1[j] - t0[j]);

    /* 3. 检查点的参考（P5 位置/速度约束） */
    float qr_e[MPC_CS_NB][MPC_NJ], dqr_e[MPC_CS_NB][MPC_NJ];
    for (int i = 0; i < MPC_CS_NB; ++i)
        traj_eval(t_apply + MPC_CS_ENDS[i] * MPC_BASE_DT, qr_e[i], dqr_e[i], tmp);

    /* 4. P3 惯量比 */
    float rho[MPC_NJ];
    mpc_inertia_ratio(q, rho);

    /* 5. P1 观测器：所有关节，每拍都更新 */
    for (int j = 0; j < MPC_NJ; ++j)
        mpc_est_update(j, qr_m[j] - q[j], dqr_m[j] - dq[j]);

    /* 6. P4: 每拍解全部关节 */
    float dtau[MPC_NJ];
    for (int j = 0; j < MPC_NJ; ++j) {
        float anchor[2], w_hat, qe[MPC_CS_NB], dqe[MPC_CS_NB];
        mpc_est_anchor(j, anchor, &w_hat);                 /* P2：前推到生效时刻 */
        for (int i = 0; i < MPC_CS_NB; ++i) { qe[i] = qr_e[i][j]; dqe[i] = dqr_e[i][j]; }
        const float u = mpc_cs_step(j, anchor, w_hat, tau_ff[j], tau_ff_end[j],
                                    rho[j], qe, dqe);
        dtau[j] = rho[j] * u;
    }

    /* 7. 装配饱和，再把真正发出去的 u 告诉观测器 */
    mpc_saturate_total(tau_ff, dtau, tau_out);
    for (int j = 0; j < MPC_NJ; ++j)
        mpc_est_commit(j, (tau_out[j] - tau_ff[j]) / rho[j]);
    tick++;
}
```

容易踩的点：

- **`rnea` 的前两个参数必须是测量的 `q`、`dq`，第三个是参考的 `ddq_ref`。**
  只有算 `tref`（时域末尾预测）时三个参数才都用参考。
- **`tau_ff_end` 是增量形式**：`tau_ff + (tref(末尾) - tref(生效时刻))`。直接用
  `tref(末尾)` 会把测量和参考之间的差（噪声、跟踪误差）带进约束。
- **`mpc_est_commit` 必须用饱和之后反算的 `u`。** 观测器只认真正发出去的量，
  所以积分作用不会在饱和时越积越多（天然抗饱和）。
- **`MPC_EST_FB / MPC_EST_CT` 要和实物的延迟一致。** 仿真里真实延迟翻倍（8 ms）
  仍然稳定，但偏差越大效果越差，上板后用 DWT 时间戳核一次。
- **跳过某一拍不解（比如超时）时**，`mpc_cs_step` 的热启动状态不受影响，
  下一拍照常调用即可；那一拍的 Δτ 沿用上一拍的 u 乘当前 ρ，`mpc_saturate_total` 兜底。

**计算量估算（没有在 H723 上实测）。** 每个关节每次 ADMM 迭代约 500 次乘加
（K⁻¹ 10×10、C 和 Cᵀ 各 20×10），25 次 × 6 关节 = 7.5 万次乘加。Cortex-M7 单精度
FMA 加上取数按 1.5~2 周期/次算，约 11~15 万周期，480 MHz 下 0.23~0.31 ms。
三次 RNEA（每次约 550 次乘法）+ CRBA + Cholesky + 观测器合计几千次运算，几十 µs。
整拍估计 < 0.4 ms，2 ms 预算有 5 倍余量。常数表 11 KB，可以整体放进 DTCM。
耗时与输入无关（定迭代、无分支收敛判断），实测一次最坏值就是全部工况的值。

---

## 7. 拿到实物之后要改什么

这些都是**参数**问题，不是代码问题——改 `mpc_config.py` 重跑第 2 节的流程即可。
按优先级：

1. **实测单拍耗时。** 用 DWT 周期计数器测 `control_tick` 整体和 `mpc_cs_step`。
   如果余量很大，可以加长时域（`MPC_BLOCKS`）或加迭代次数（`ADMM_ITERS`）。
2. **摩擦参数。** `FRICTION_COULOMB/VISCOUS` 是暂定值，台架测完替换。
   手腕的观测器参数（`OBS_*`）是按仿真里的摩擦调的，要一起重调。
3. **延迟。** 核对 `EST_FB_DELAY_TICKS / EST_CTRL_DELAY_TICKS`。
4. **限位。** `JOINT_Q_MIN/MAX` 现在按要求写死 ±3.14，速度限位取 URDF 的 velocity
   （3 rad/s）。实物的机械限位和安全速度确定后改这里。
5. **负载和轨迹。** `PAYLOAD_*`、`Q_GOAL` 是暂定值；`Q_GOAL` 会进惯量设计，改了要重新生成。

---

## 8. 已知的坑和未完成项

**坑：**

- **numpy 的二维矩阵乘矩阵（`A @ B`，以及 `np.linalg.*`）在这个环境里会原生崩溃**
  （Windows 异常码 0xc06d007f，进程直接退出，`try/except` 抓不到），矩阵乘向量不会。
  推测是 BLAS 的矩阵乘接口没装好，没有进一步确认。自己加代码时用 `dyn_model._mm()`、
  `cmpc._mv()`、`cmpc._inv_spd()`。
- `MAKE_PLOT = True` 时 matplotlib 在 robotics 环境里会原生崩溃。看图用
  `plot_npz.py` 生成离线 HTML。
- 脚本别起和标准库同名的文件名（比如 `bisect.py`），会把标准库挡掉。
- 只跑 `sim.py` 不跑 `gen_code.py`（或反过来）会得到不一致的两套参数，见第 4 节。
- `generated/` 每次生成重建，往里面加手写文件会被覆盖。
- 生成器里写 C 常数表时，f-string 和普通字符串的花括号写法不一样，写错了
  gcc 只报一个没有开关名的 warning、常数表内容却是错的。`build.sh` 会把
  这类 warning 拦成错误。
- 新求解器的仿真比旧的慢（30 s 仿真约 80 s，旧的 11 s），因为 Python 里
  每拍要跑 6 × 25 次迭代。C 里没有这个问题。

**未完成项：**

- 单拍耗时没有在 H723 上实测（第 6 节是估算）。
- 齿隙、电流环延迟没建模；摩擦只有库伦 + 粘性。
- 单片机工程还没接新接口：现在的 `generated/` 只在 PC 上编译和回归过。
