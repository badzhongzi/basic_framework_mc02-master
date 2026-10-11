"""仿真和 C 代码生成共用的 MPC 配置。

修改这里的参数后，分别运行 sim.py 和 gen_code.py，二者会使用同一套
采样周期、权重、错峰策略和 OSQP 配置。
"""

URDF_PATH = "C:/Users/Badzhongzi/PycharmProjects/PythonProject/openarm.urdf"

# 物理仿真/任务的基础周期。每个关节的 MPC 更新周期由调度策略决定。
BASE_DT = 2.0e-3

# 预测步数。单关节采样周期 6 ms，故预测时域为 N*6 ms = 60 ms。
N = 10

# 残差模型的惯量取法。详见 dyn_model.py 的说明。
#   "effective_min" 工作空间内 M_eff=1/(M^-1)_jj 的最小值（推荐）
#   "effective_q0"  只在 q=0 处取 M_eff
#   "diagonal_q0"   M_jj，旧行为；实测关节 2/3 闭环主极点 1.79/5.97，会发散
# 用 M_jj 会把反馈增益低估 M_jj*(M^-1)_jj 倍，openarm.urdf 在 q=0 处这个
# 倍数是 [2.0, 19.5, 25.7, 2.5, 3.2, 1.5]，关节 2/3 因此 bang-bang 抖动。
INERTIA_MODE = "effective_min"
INERTIA_SAMPLES = 4000          # 采样构型数；改动它会改变 b1，sim 和 codegen 必须一致
INERTIA_SEED = 0

# 仿真参考轨迹的起止点和时长。惯量采样会把 Q_GOAL 也作为关注构型加进去，
# 所以 sim.py 和 gen_code.py 必须共用这里的值，否则两边生成的 b1 会不一致。
# 注意：Q_GOAL[2]=5.3 超出 URDF 的 ±3.14 位置限位，max|dq_ref|=1.807 也超过
# URDF 的 velocityLimit=1.0。URDF 里这两组限位看着像占位值，先按原样保留。
Q_INIT = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
Q_GOAL = [1.5, -1.4, 5.3, -3.6, 2.4, -1.2]
T_TRAJ = 5.5
T_SIM = 30.0

# 扰动注入时刻 [s] 和注入的位置跳变 [rad]。放在轨迹结束之后，
# 这样"恢复时间"只反映扰动抑制能力，不会被轨迹跟踪误差污染。
PERTURB_T = 8.0
PERTURB_DQ = [12.55, -15.33, 11.40, 12.0, 18.0, 19.0]

# 权重按 openarm.urdf 的实际等效惯量选定
# （M_eff 工作空间最小值 [0.0300, 0.0237, 0.0097, 0.0120, 0.0142, 0.0067] kg*m^2）。
# 该组权重对应 K_pos 约 17~62 Nm/rad、K_vel 约 1.2~4.6 Nm*s/rad，
# 6 ms 零阶保持下全工作空间最坏主极点 0.967，沉降到 2%：关节 1 最坏 0.69 s
# （它的 M_eff 跨 13 倍，按最小值设计所以最大惯量处偏慢），其余五个 0.26 s。
# Q_POS/R_WEIGHT = 4e4，使 float32 下 P 的条件数保持在 4e4，留足精度余量。
Q_POS = 2000.0
Q_VEL = 5.0
R_WEIGHT = 0.05

# 单步力矩修正上限 [Nm]。实物电机参数未定，这里取和前馈峰值同量级的占位值：
# 参考轨迹上 max|tau_ff| 约 9.1 Nm，故总力矩最坏约 21 Nm。
# 拿到电机额定/峰值力矩后应改成 tau_peak - max|tau_ff| 的安全余量。
# 位置误差超过约 DTAU_MAX/K_pos（本组增益下 0.19~0.72 rad）后该约束起作用，
# 此时 MPC 才真正优于"PD 输出后直接限幅"。
#
# 待办：总力矩 |tau_ff + Δτ| 目前无界。DTAU_MAX 只管住了 Δτ，tau_ff 里的
# 科氏/离心项按 dq^2 增长，大扰动恢复时能顶得很高。只切换误差钳位实测：
#   钳位关闭  max|dq|=130.1 rad/s, max|tau|=[610, 631, 686, 186, 177, 42] Nm
#   钳位 f=3  max|dq|= 30.8 rad/s, max|tau|=[ 64,  73,  70,  21,  18, 12] Nm
# 钳位顺带把峰值力矩压了一个数量级（恢复过程变单调，不再来回抽）。
# 但两组的速度都远超任何真实机械臂，12~19 rad 阶跃本身也不物理——
# 下一步加总力矩约束时应同时把扰动量级改成实际值，否则是在对着一个虚构工况调参。
DTAU_MAX = 12.0

# 每个基础周期求解几个关节。1 表示轮询求解，6 轴完整轮询周期为 6*BASE_DT。
JOINTS_PER_TICK = 2

# None 表示根据错峰策略自动计算；设置为浮点数可显式指定单关节 MPC 采样周期。
MPC_DT_OVERRIDE = None

# 完整 RNEA(q_actual, dq_actual, ddq_ref) 前馈下，默认不重复加入重力梯度项。
USE_GRAVITY_GRADIENT_MODEL = False

# 结果画图。robotics 环境的 matplotlib 渲染后端是坏的（canvas.draw() 原生崩溃，
# 无法用 try/except 捕获），设为 False 可跳过画图，统计数据仍会打印并存入 npz。
MAKE_PLOT = True

# OSQP 使用 0.5*z.T*P*z + q.T*z。设为 2 时对应未省略 0.5 的二次代价。
QP_COST_FACTOR = 2.0

# rho 实测：0.1 在 1.4 rad 扰动下要 650 次迭代，0.01 撞 max_iter 且解越界；
# 2.0 时最坏 75 次、均值 52 次，是这组权重下的最优值。
OSQP_RHO = 2.0

# 收敛容差。Δτ 的量级是 DTAU_MAX=12 Nm，1e-3 绝对容差相当于 1e-4 相对精度，
# 对执行器没有任何影响。实测 1e-4 -> 1e-3 把 iter_max 从 70 降到 55，
# e轨迹 完全不变（3.67e-5）。
OSQP_EPS_ABS = 1.0e-3
OSQP_EPS_REL = 1.0e-3

# max_iter 是硬性截止保护，不是调参杠杆：配好钳位后 iter_max=55，
# 把上限设到 60 或 100 结果完全相同（该约束根本不激活）。
# 取 100 给未建模工况留一倍余量，配合 USE_UNCONVERGED_ITERATE 保证不会错过 tick。
OSQP_MAX_ITER = 100
OSQP_WARM_START = True

# OSQP 终止检查间隔。默认 25 会把报出的 iter 量化成 25 的倍数，
# 真实收敛点可能远早于此。实测 25 -> 5 把 iter_mean 从 28.1 降到 17.8。
# 不取 1：残差检查要算 A*z 和 P*z 的范数，单次开销和一次 ADMM 迭代同量级，
# 每轮都检查会让每次迭代的实际耗时接近翻倍。ct=1 只把 iter_max 从 55 降到 52，
# 净效果是变慢。取 5 时检查开销摊薄到约 20%。
OSQP_CHECK_TERMINATION = 5

# ================================================================
# 实时性预算
# 目标板 STM32H7 @480MHz。JOINTS_PER_TICK=1、BASE_DT=1ms，所以每个 1 ms
# tick 内要跑完一次 6 自由度 RNEA 加一次完整 QP 求解——预算是 1 ms，不是 6 ms。
# nz=nc=3N=30 的 float32 嵌入式 OSQP 单次迭代约 1~3 us，留出 RNEA 和中断余量后
# 迭代数上限定在 100。
#
# 调参结果（sweep_rt.py，12~19 rad 阶跃扰动）：iter_max 1000 -> 55，iter_mean 30.8 -> 14.3。
# 按 3 us/迭代的悲观值算，最坏 55*3*1.2 = 198 us，加 6 自由度 RNEA 约 20~40 us，
# 单 tick 最坏约 240 us，对 1 ms 预算有 4 倍余量。这个数还要在实物 H7 上实测确认。
# 三个生效的杠杆按贡献排序：误差钳位（主）> check_termination > eps。
# max_iter 不是杠杆，只是截止保护。
# ================================================================

# 误差钳位——压迭代数的主杠杆。|e| 远超线性范围时 MPC 的答案必然是顶满 DTAU_MAX，
# 但这种输入会让全部 N 个控制约束同时激活，正是 ADMM 最慢的工况：
# 不钳位时关节 6 要 1000 次迭代去求一个已知的答案。
# 把喂进 QP 的锚点误差钳到线性范围的若干倍，QP 就始终留在良态区。
# 上限由 LQR 增益反推：E_MAX_j = factor*DTAU_MAX/K_pos_j，
#                      DE_MAX_j = factor*DTAU_MAX/K_vel_j。
#
# 钳位确实会改变饱和区的控制行为，不是无损的。e_q 和 e_dq 分别独立钳位，
# 二者在 LQR 里符号相反时（高速接近目标）贡献会部分抵消，钳位改变了它们的比值，
# 于是 bang-bang 的切换时刻也变了。实测 12~19 rad 阶跃扰动下：
#   factor  iter_max  饱和%   恢复s
#   None      1000     8.83   1.756
#   10         425     6.64   1.931
#   5          150     5.74   1.953
#   3           55     2.79   2.008     <- 采用
#   2           50     1.50   2.337
#   1           50     0.09   4.034
# 取 3：iter_max 降到 1/18，恢复时间只退化 14%（factor=2 就要退化 33%）。
# 全程 e轨迹 恒为 3.67e-5，钳位对轨迹跟踪零影响——运动段误差远在钳位范围内，
# 只有大扰动瞬态才会触发（占全部求解的 6.5%）。
# None 表示不钳位。嵌入式侧必须用同一组上限，否则仿真不可信。
E_CLAMP_FACTOR = 3.0

# max_iter 用尽时如何处置：
#   True  = 直接用 OSQP 返回的当前迭代点（钳到 ±DTAU_MAX 后使用）
#   False = 沿用上一周期的 Δτ（旧行为）
# 大扰动瞬态下 hold 一个过期值，比用未收敛但方向已经正确的迭代点更糟。
USE_UNCONVERGED_ITERATE = True
