# real/ 使用说明

real/
├── mpc_config.py          ← 唯一参数入口（改参数只改这里）
├── dyn_model.py           ← 等效惯量、LQR增益、钳位上限计算
├── gen_code.py            ← 生成 C 代码的脚本
├── sim.py                 ← 闭环仿真（验证精度/迭代/恢复时间）
├── sim_core.py            ← 仿真内核
├── sweep_rt.py            ← 参数扫描（找实时性平衡点）
├── USAGE.md               ← 完整使用说明
├── ctest/                 ← C 侧 float32 数值回归
│   ├── build.sh           ← 编译+运行回归测试
│   ├── test_qp.c          ← QP 回归：C vs Python Δτ 偏差
│   ├── test_rnea.c        ← RNEA 回归：CasADi float32 vs pinocchio float64
│   ├── dump_qp.py         ← 导出 QP 用例
│   └── dump_rnea.py       ← 导出 RNEA 用例
└── generated/             ← 产物（绝对不要手改）
    ├── mpc_6axis.h        ← MPC 6轴统一接口
    ├── mpc_6axis.c        ← MPC 实现（build_vectors / update / solve / get_dtau）
    ├── rnea_6axis.h       ← CasADi 生成的 RNEA 接口
    ├── rnea_6axis.c       ← CasADi 生成的 RNEA 实现（前馈力矩）
    └── generated_solver/
        ├── common/        ← OSQP 公共层（osqp_api.c, qdldl.c 等）
        ├── joint0/        ← 关节0 的 OSQP workspace（预分解 P、A）
        │   ├── mpc_j0workspace.c/h  ← 预分配的 solver 结构体
        │   └── emosqp.c            ← 嵌入式 OSQP 入口
        ├── joint1/ ... joint5/      ← 同上，6个关节各一份
        └── joint5/

这套脚本做一件事：从 URDF 出发，设计并验证一个「完整 RNEA 前馈 + 6 个独立关节
MPC 错峰求解」的控制器，然后生成 STM32H7 能直接编译的 C 代码。

控制律是

```text
tau = tau_ff + Δτ
tau_ff = RNEA(q_实际, dq_实际, ddq_参考)     全自由度，一次算 6 个
Δτ     = 每关节独立 MPC 的修正量，只管自己那一路
```

前馈是主项，MPC 只负责补模型误差和抑制扰动。分工的量级：参考轨迹上
`max|tau_ff|` 约 9.1 Nm，大扰动恢复时总力矩峰值到 64~73 Nm，而 MPC 的权限
只有 ±12 Nm。这个分工决定了后面所有设计取舍——尤其是残差模型可以线性化。

---

## 0. 环境

```text
python  D:/Minforge/envs/robotics/python.exe     pinocchio 4.1.0 / osqp 1.1.3 / casadi 3.7.2
gcc     只有跑 ctest/ 才需要（MinGW 或任意 C11 编译器）
```

下面所有命令都在 `real/` 目录下执行。

---

## 1. 文件职责

| 文件 | 干什么 | 要不要手改 |
|---|---|---|
| `mpc_config.py` | **唯一的参数入口**，仿真和代码生成共用 | 改参数只改这里 |
| `dyn_model.py` | 等效惯量、LQR 增益、闭环极点、钳位上限 | 不用 |
| `sim_core.py` | 闭环仿真内核 + 统计定义 | 不用 |
| `sim.py` | 跑一次完整仿真，打印统计、存 npz、画图 | 不用 |
| `sweep_rt.py` | 参数扫描，找实时性和精度的平衡点 | 加 case 时改 |
| `gen_code.py` | 生成 `generated/` 下的全部 C 代码 | 不用 |
| `ctest/` | C 与 Python 的 float32 数值回归 | 不用 |
| `generated/` | 产物，每次生成整目录重建 | **绝对不要** |

---

## 2. 典型流程

```bash
# 1) 改参数
#    编辑 mpc_config.py

# 2) 仿真验证：看精度、迭代数、恢复时间
D:/Minforge/envs/robotics/python.exe -u sim.py

# 3) 生成 C 代码
D:/Minforge/envs/robotics/python.exe -u gen_code.py

# 4) 验证生成的 C 和 Python 数值一致
cd ctest
D:/Minforge/envs/robotics/python.exe -u dump_qp.py
D:/Minforge/envs/robotics/python.exe -u dump_rnea.py
sh build.sh
```

第 2 步和第 3 步**必须成对重跑**。原因见第 4 节。

---

## 3. 架构一页纸

**错峰调度。** `BASE_DT = 1 ms`，`JOINTS_PER_TICK = 1`，所以每个 1 ms 的 tick 里：

- 算一次全自由度 RNEA（6 个关节的 `tau_ff` 一起出来）
- 只解 **一个** 关节的 QP，`j = tick % 6`
- 其余 5 个关节沿用上次算出的 `Δτ`（hold）

因此单关节的 MPC 更新周期是 `MPC_DT = 6 ms`，但**实时预算是 1 ms 而不是 6 ms**。
这一点很容易搞错：6 ms 只是某个关节被重新计算的间隔，每个 1 ms 都有一次完整求解。

**锚点误差。** MPC 的状态不是普通跟踪误差，而是

```text
e_anchor,k = x_ref(t0) - x(t0 + k·MPC_DT)      参考锁在 t0 不动
delta_k    = x_ref(t0) - x_ref(t0 + k·MPC_DT)  参考自己的偏移，delta_0 恒为 0
```

两者一减就是未来时刻的普通跟踪误差。递推是
`e_(k+1) = A·e_k + B·u_k + delta_(k+1) - A·delta_k`，其中 k=0 那一行额外加上
`A·e_anchor`。这套写法在 Python、生成器、C 三处逐行对应，数值差恰好 0。

**当前关键量**（改了 `mpc_config.py` 记得对照更新）：

| 量 | 值 | 来自 |
|---|---|---|
| 预测步数 N | 10 → 时域 60 ms | `N` |
| QP 规模 | `nz = nc = 3N = 30` | 推出 |
| 残差模型惯量 | 工作空间 **最小** 等效惯量 `1/(M⁻¹)_jj` | `INERTIA_MODE` |
| 闭环主极点 | 设计点 0.914（沉降 0.26 s）；全工作空间最坏 0.967（关节 1，0.69 s） | `dyn_model.closed_loop_pole` |
| 修正权限 | ±12 Nm | `DTAU_MAX` |
| 最坏迭代数 | 55（上限 100） | 实测 |

为什么惯量用 `1/(M⁻¹)_jj` 而不是 `M_jj`：完整前馈下真实闭环是
`ddq = ddq_ref + M(q)⁻¹Δτ`，单独施加 `Δτ_j` 得到的是 `(M⁻¹)_jj·Δτ_j`。用 `M_jj`
会把反馈增益低估 `M_jj·(M⁻¹)_jj` 倍，本 URDF 在 q=0 处这个倍数高达 25.7，
关节 2/3 会直接发散。取工作空间最小值是因为 `Ad/Bd` 在 C 里是编译期常数，
必须一个值走遍全工作空间——低估惯量会过增益发散，高估只是变慢但仍稳定。
详见 `dyn_model.py` 的模块注释。

---

## 4. 改参数的规则

**所有参数只在 `mpc_config.py` 里改。** `sim_core.py` 的 `Settings` 会校验键名，
拼错的名字会立刻报错而不是被静默忽略。

**`sim.py` 和 `gen_code.py` 必须成对重跑**，只跑一个会得到一个「仿真验过的参数
和板上跑的参数不是同一套」的系统。耦合点在惯量设计：

```text
Q_GOAL, Q_INIT, INERTIA_MODE, INERTIA_SAMPLES, INERTIA_SEED
    -> dyn_model.design_inertia()  取工作空间内 M_eff 的最小值
    -> b1 = -DT / M_eff            进入 C 侧的 MPC_B1[]
    -> 同时决定 LQR 增益、钳位上限 MPC_E_MAX/MPC_DE_MAX
```

也就是说，改一个看起来只跟仿真轨迹有关的 `Q_GOAL`，会改变生成的 C 常数。
`design_inertia` 把 `Q_GOAL` 当作重点关注构型显式加进采样集，就是为了这个。

改完之后至少确认三件事（都在 `sim.py` 的输出里）：

| 看什么 | 期望 | 不对的话 |
|---|---|---|
| 闭环主极点 | 全部 < 1，最好 < 0.97 | 惯量或权重不对，会发散 |
| `iter_max` | ≤ `OSQP_MAX_ITER` | 实时性不够，回第 5 节调 |
| `e稳态` / 未恢复数 | 0 / 0 | 有关节没稳住 |

`E_CLAMP_FACTOR`、`OSQP_*` 这些只影响求解器，不影响 `b1`，但它们也进 C 侧
（钳位上限是编译期常数，`check_termination` 写进 workspace），所以同样要重跑
`gen_code.py`。结论：**改任何参数，都把第 2、3、4 步重跑一遍**，反正总共几十秒。

---

## 5. 调实时性：`sweep_rt.py`

用途：换了板子、换了 N、换了权重之后，重新找一组能塞进 tick 的求解器配置。

```bash
D:/Minforge/envs/robotics/python.exe -u sweep_rt.py     # 23 组配置，几分钟
```

它把四个杠杆逐个扫一遍再跑几组组合，每行给 `iter_mean / iter_max / 未收敛 /
钳位% / e轨迹 / e稳态 / 饱和% / 恢复s / 未恢复`，最后按判据挑出最优通过配置。
判据写在文件头部：`iter_max ≤ 100`、`e轨迹 ≤ 1e-4`、恢复时间相对基线退化 ≤ 20%、
未恢复关节数为 0。

**杠杆按有效性排序：**

1. `E_CLAMP_FACTOR` —— 唯一能改一个数量级的杠杆（1000 → 55 次迭代）。代价是恢复变慢。
2. `OSQP_CHECK_TERMINATION` —— 主要改善 `iter_mean`。默认 25 会把报出的迭代数
   量化成 25 的倍数，看着比真实收敛点大很多。不要取 1：残差检查本身约等于一次
   ADMM 迭代的开销，ct=1 只把 `iter_max` 从 55 降到 52，净效果更慢。
3. `OSQP_EPS_ABS/REL` —— 效果很小。Δτ 量级 12 Nm，1e-3 已经是 1e-4 相对精度，
   对执行器没有意义，再收紧纯属浪费迭代。
4. `OSQP_MAX_ITER` —— **不是杠杆**，是硬性截止保护。配好钳位后设 60 和 100
   结果完全相同（约束根本不激活）。取 100 只是给未建模工况留余量。

`OSQP_RHO` 没进扫描，因为它和权重强耦合，改了权重要单独重调。当前 2.0：
0.1 要 650 次迭代，0.01 撞上限且解越界。

**钳位不是无损的。** 一度以为「误差被钳位后输出仍然饱和，所以控制行为不变」——
这是错的。`e_q` 和 `e_dq` 分别独立钳位，二者符号相反时（高速接近目标）它们在
LQR 里的贡献会部分抵消，钳位改变了这个比值，于是 bang-bang 的切换时刻也变了。
证据是饱和% 和恢复时间随 factor 收紧单调变化，表见 `mpc_config.py`。

---

## 6. 验证生成的 C：`ctest/`

Python 仿真全程 float64，板上是 float32。这两套代码算的是不是同一件事，必须验，
不能靠「应该一样」。

```bash
cd ctest
D:/Minforge/envs/robotics/python.exe -u dump_qp.py     # 导出 1150 个 QP 用例
D:/Minforge/envs/robotics/python.exe -u dump_rnea.py   # 导出 3410 个 RNEA 用例
sh build.sh                                            # 编译并跑两个回归
```

`build.sh` 把 `generated/` 下的全部源文件和 OSQP 的 common 层一起编译，
所以它同时也是**生成代码能不能编过**的检查。上板前跑一次，编译错误在这里就暴露。

**`test_qp`** —— 回放仿真第 7000~8150 步（稳态起步，让 C 的冷启动接近 Python 的
热启动状态），按记录顺序逐个喂进去以保证 warm start 可比。喂的是**未钳位**的原始
锚点误差，这样 C 侧内部的钳位逻辑也一并被验到。判据：状态码无异常、钳位结果一致、
Δτ 偏差 ≤ 2% DTAU_MAX。当前结果：异常 0、钳位偏差恰好 0、Δτ 平均偏差 2.7e-5 Nm、
最坏 0.0107 Nm（0.089% DTAU_MAX，发生在饱和点 C=-12.000 对 Py=-11.989）。

**`test_rnea`** —— 生成的 `rnea_6axis.c`（CasADi 符号展开 + float32）对
`pin.rnea`（float64）。判据不取抽象浮点容差而取物理量：前馈误差对 MPC 来说
和外部扰动无法区分，所以必须远小于修正权限，取 1% DTAU_MAX = 0.12 Nm。
分三组统计，好让「实际工况没问题但极端工况精度崩」这种情况看得出来：

| 组 | 用例 | 内容 | 最大误差 |
|---|---|---|---|
| gravity | 102 | 纯重力静止 | 2.0e-6 Nm |
| sim | 2308 | 闭环实际经历的 (q, dq, ddq_ref) | 5.8e-5 Nm |
| extreme | 1000 | 随机构型 + dq 拉到 130 rad/s 压科氏项 | 9.4e-4 Nm |

extreme 组豁免绝对判据（那里 tau 本身到 3883 Nm，也不是真实工况），但相对误差
必须 ≤ 1e-3。全局最坏 9.4e-4 Nm = 0.0078% DTAU_MAX，比判据宽 128 倍。

相对误差从 1.1e-6（gravity）到 1.3e-4（extreme），随力矩量级单调增长，
量级上就是 float32（eps ≈ 1.2e-7）在一棵很深的表达式树上累积的结果。
所以这是舍入而不是结构性错误——模型、关节顺序、重力符号任何一处错了，
都会给出 O(1) Nm 的误差而不是 1e-4。

---

## 7. C 侧集成

要编译进工程的文件：

```text
generated/mpc_6axis.c            调度用的薄封装 + 在线向量构造 + 钳位
generated/rnea_6axis.c           CasADi 展开的 6 自由度 RNEA
generated/generated_solver/      OSQP 嵌入式代码，6 个关节各一个 workspace
    common/src/*.c               求解器核心
    joint0..joint5/*.c           每个关节的 workspace（Ad/Bd 已烧成常数）
```

include 路径见 `ctest/build.sh` 里的 `$INC`，照抄即可。
`generated_solver/sources.txt` 是生成器写的文件清单，配 Makefile 时可以直接用。

**需要你自己提供的只有一个东西：轨迹求值函数。** 它要能在任意时刻给出
`(q_ref, dq_ref, ddq_ref)`，而且要能求 `t0 + k·MPC_DT`（k 到 N）的未来点，
因为 δ 序列就是从这里来的。仿真里用的是五次多项式，板上换成你的插值器都行，
MPC 只关心它连续可导。

下面这段是**示意**，不是已验证的代码。当前 Python 和 C 的一致性只验到了单次
求解这一层（第 6 节），整个 tick 循环——调度、hold 另外五路 Δτ、
`tau = tau_ff + Δτ` 的装配——目前只存在于 Python 仿真里。上板前建议把它也做成
闭环回归（见第 9 节）。

```c
#include <stdint.h>
#include "mpc_6axis.h"
#include "rnea_6axis.h"

static float    dtau_hold[MPC_NJ];   /* 未轮到的关节沿用上次的修正量 */
static uint32_t tick;

/* 1 kHz 定时中断里调用。tau_out 送给电机。 */
void control_tick(const float q[MPC_NJ], const float dq[MPC_NJ],
                  float tau_out[MPC_NJ])
{
    const float t = tick * MPC_BASE_DT;

    /* --- 前馈：全自由度，每个 tick 都算 --- */
    float q_ref[MPC_NJ], dq_ref[MPC_NJ], ddq_ref[MPC_NJ], tau_ff[MPC_NJ];
    traj_eval(t, q_ref, dq_ref, ddq_ref);            /* 你自己的轨迹求值 */

    const casadi_real *arg[3] = { q, dq, ddq_ref };  /* 注意用实际 q/dq */
    casadi_real *res[1] = { tau_ff };
    rnea(arg, res, 0, 0, 0);

    /* --- 反馈：本 tick 只更新一个关节 --- */
    const int j = (int)(tick % MPC_NJ);
    const float anchor[2] = { q_ref[j] - q[j], dq_ref[j] - dq[j] };

    float delta[MPC_N + 1][2] = {{0}};               /* delta[0] 恒为 0 */
    for (int k = 1; k <= MPC_N; ++k) {
        float qk[MPC_NJ], dqk[MPC_NJ], ddqk[MPC_NJ];
        traj_eval(t + k * MPC_DT, qk, dqk, ddqk);
        delta[k][0] = q_ref[j] - qk[j];
        delta[k][1] = dq_ref[j] - dqk[j];
    }

    float qv[MPC_NZ], l[MPC_NC], u[MPC_NC];
    mpc_build_vectors(j, anchor, delta, qv, l, u);   /* 内部已钳位 */
    if (mpc_update_joint(j, qv, l, u) == 0) {
        mpc_solve_joint(j);
        dtau_hold[j] = mpc_get_first_dtau(j);        /* 已钳到 ±DTAU_MAX */
    }   /* update 失败就保留上次的值，不要写 0——突然撤掉修正比过期值更糟 */

    for (int i = 0; i < MPC_NJ; ++i)
        tau_out[i] = tau_ff[i] + dtau_hold[i];
    tick++;
}
```

三个容易踩的点：

- **`rnea` 的前两个参数必须是实际的 `q`、`dq`，第三个是参考的 `ddq_ref`。**
  传参考位置进去就变成了纯开环前馈，MPC 要去补的误差会大一个数量级。
- **`delta[0]` 必须是 0**，这是锚点定义的一部分，不是可省略的初始化。
- **`mpc_get_first_dtau` 的返回值直接用，不要再判状态码丢弃。** 仿真里
  `USE_UNCONVERGED_ITERATE=True`，未收敛时也采用当前迭代点；C 侧改成
  「不收敛就 hold」会和仿真行为不一致，而且大扰动下更差。

---

## 8. 拿到实物之后要改什么

这些都是**参数**问题，不是代码问题——改 `mpc_config.py` 重跑第 2~4 步即可。
按优先级：

1. **电机峰值力矩 → `DTAU_MAX`。** 现在的 12 Nm 是占位值，按「和前馈峰值同量级」
   拍的。正确做法是 `DTAU_MAX_j = tau_peak_j - max|tau_ff_j| - 安全余量`，
   而且应该**每个关节一个值**（现在是六轴共用一个标量，要改成数组，
   `dyn_model.clamp_limits` 和生成器都按数组处理起来不难）。
2. **总力矩约束。** 当前只管住了 `Δτ`，`|tau_ff + Δτ|` 仍然无界——`tau_ff` 里的
   科氏/离心项按 `dq²` 增长。实测在 12~19 rad 的阶跃扰动下峰值到
   `[64, 73, 70, 21, 18, 12] Nm`（钳位关掉的话到 686 Nm）。要么在 QP 里把
   `tau_ff` 作为常数加进约束，要么在输出级做饱和 + 抗饱和处理。
3. **真实扰动量级。** 现在的 `PERTURB_DQ` 是 12~19 rad/s 的速度跳变，纯粹为了
   压测求解器，任何真实机械臂都到不了。换成实际能遇到的量级（碰撞、负载突变）
   之后，`E_CLAMP_FACTOR` 大概可以放松，恢复时间那 14% 的退化就不用付了。
   **第 2 项和第 3 项应该一起做**，否则是在对着一个虚构工况调参。
4. **URDF 限位。** `openarm.urdf` 里的位置限位 ±3.14 和 `velocityLimit=1.0`
   看着是占位值：当前 `Q_GOAL[2]=5.3` 超了位置限位，`max|dq_ref|=1.807` 超了速度
   限位，但仿真跑得好好的。拿到真实限位后要重新确认轨迹合法，并考虑把限位作为
   状态约束加进 QP（现在 QP 里只有控制量的箱约束）。
5. **实测单 tick 耗时。** 预算估算是 55 迭代 × 3 µs × 1.2 ≈ 198 µs 加 RNEA
   20~40 µs，对 1 ms 有 4 倍余量。这个数**没有在真实 H7 上测过**，
   用 DWT 周期计数器测 `mpc_solve_joint` 和 `rnea` 的最坏耗时，
   然后回头核对 `OSQP_MAX_ITER`。
6. **电机/减速比参数。** URDF 的惯量和质量若和实物有差异，改 URDF 重跑即可，
   `M_eff` 会自动重新采样。关节摩擦目前完全没建模，它会表现为一个常值扰动，
   由 MPC 吸收——如果实测发现稳态误差，优先怀疑这里。
7. **计划之后把仿真频率和控制率频率分开**
---

## 9. 已知的坑和未完成项

**坑：**

- `MAKE_PLOT = True` 时 matplotlib 在 robotics 环境里会原生崩溃
  （`canvas.draw()` 挂在 C 层，`try/except` 抓不住，整个进程直接没）。
  批量跑或者在 CI 里跑，先设 `MAKE_PLOT = False`——统计照样打印，数据照样存 npz。
- 同一个环境里 `np.linalg.inv` 偶发原生崩溃。`dyn_model` 里用的是 `solve`
  路径，如果自己加代码，别引入新的 `inv` 调用。
- 只跑 `sim.py` 不跑 `gen_code.py`（或反过来）会得到不一致的两套参数，见第 4 节。
- `generated/` 每次生成整目录重建，往里面加手写文件会被覆盖。

**未完成项：**

- **没有 C 侧闭环回归。** 第 6 节验的是单次求解，tick 循环只在 Python 里存在。
  把第 7 节那段和 `rnea` 一起在 PC 上跑一条完整轨迹，对 `sim.py` 的 `q/dq/tau`
  逐步对数，就能把「每次求解都对」升级成「整条轨迹都对」。这是上板前最值得补的
  一件事，也是唯一一个不依赖实物的剩余缺口。
- **`Δddq` 架构分支。** 让 MPC 输出加速度修正而不是力矩修正，可以天然规避
  等效惯量要取全工作空间最小值的保守性（修正量会被 `M(q)` 自动缩放）。
  代价是每个 tick 要多算一次 `M(q)` 的相关量。当前架构够用，这条按需再说。
- **状态约束。** QP 里只有 `|Δτ| ≤ DTAU_MAX`，位置和速度限位没有作为约束进去。
- **摩擦、齿隙、电流环延迟**都没建模。

**δ 机制的冗余。** `delta_seq` 传了 `N+1` 个点，但 `delta[0]` 恒为 0，
递推里也只用到 `delta[k+1] - A·delta[k]`，理论上可以在生成端预压缩。
保留现在的形式是刻意的：Python、生成器、C 三处逐行对应，改错的风险大于省下的
那几十个浮点乘法。

