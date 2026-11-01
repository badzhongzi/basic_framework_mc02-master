#ifndef MPC_6AXIS_H
#define MPC_6AXIS_H
#ifdef __cplusplus
extern "C" {
#endif

#define MPC_NJ  6
/* 每拍（MPC_BASE_DT）解全部 MPC_NJ 个关节，预测步长也是 MPC_BASE_DT（P4）。 */
#define MPC_BASE_DT  0.002f

/* ================= P4/P5/P7: 消元 + 定迭代 ADMM =================
 * 预测时域 MPC_CS_NS 拍，分成 MPC_CS_NU 块（块内控制量不变），
 * 每块末尾检查一次位置/速度约束，共 MPC_CS_NB 个检查点。
 * 每次求解固定迭代 MPC_CS_ITERS 次，耗时是常数。 */
#define MPC_CS_NU     10
#define MPC_CS_NB     10
#define MPC_CS_NS     30
#define MPC_CS_ITERS  25
/* 块长和块末尾的步号（1 起）。调用方要按 MPC_CS_ENDS 取参考预览：
 * 第 i 个检查点的参考时刻 = 力矩生效时刻 + MPC_CS_ENDS[i] * MPC_BASE_DT。 */
extern const unsigned char MPC_CS_BLOCKS[MPC_CS_NU];
extern const unsigned char MPC_CS_ENDS[MPC_CS_NB];
#define MPC_CS_ENDS_LIST { 1, 2, 3, 5, 7, 10, 14, 18, 24, 30 }

/* 关节限位（已扣余量 POS/VEL_LIMIT_MARGIN）。q ∈ [Q_LO, Q_HI]，|dq| <= V_HI。 */
extern const float MPC_Q_LO[MPC_NJ];
extern const float MPC_Q_HI[MPC_NJ];
extern const float MPC_V_HI[MPC_NJ];

/* 修正量上限 [Nm]，逐关节。各关节的等效惯量相差三个数量级，
 * 共用一个标量会让手腕的钳位上限失去意义，所以这里是数组。
 * MPC_DTAU_MAX_WORST 是六轴里的最大值，只用于给调用方开数组、做量级断言。 */
extern const float MPC_DTAU_MAX[MPC_NJ];
#define MPC_DTAU_MAX_WORST  1.50000000e+01f
/* 花括号初始化式，给只包含本头文件、不链接 mpc_6axis.c 的消费者
 * （例如 ctest/test_rnea）用：static const float lim[MPC_NJ] = MPC_DTAU_MAX_LIST; */
#define MPC_DTAU_MAX_LIST { 8.00000000e+00f, 1.50000000e+01f, 1.20000000e+01f, 3.00000000e+00f, 3.00000000e+00f, 1.50000000e+00f }

/* 关节力矩绝对上限 [Nm]（电机能出的力，来自 URDF 的 effort）。
 * QP 把 |tau_ff + Δτ| 约束在这个值以内——不是只约束 Δτ。
 * mpc_cs_step() 需要当前和时域末尾的 tau_ff 才能算出这个边界。 */
extern const float MPC_EFFORT_MAX[MPC_NJ];
#define MPC_EFFORT_MAX_LIST { 2.80000000e+01f, 4.80000000e+01f, 4.80000000e+01f, 1.25000000e+01f, 2.80000000e+01f, 1.25000000e+01f }

/* 摩擦前馈补偿的系数。板上必须用这一组，否则总力矩约束里的 tau_ff
 * 和实际下发的不是一个东西。 */
extern const float MPC_FRIC_FC[MPC_NJ];
extern const float MPC_FRIC_FV[MPC_NJ];
#define MPC_FRIC_SCALE  8.00000000e-01f
#define MPC_FRIC_DQ_EPS 2.00000000e-02f

/* 摩擦前馈：tau_out[j] = SCALE*(FC[j]*tanh(dq[j]/DQ_EPS) + FV[j]*dq[j])。
 * dq 用的是测量值（带噪声），所以 SCALE < 1，不补满。
 * 调用方要把它加进 tau_ff，再把 tau_ff 传给 mpc_cs_step()。 */
void mpc_friction_ff(const float *dq, float *tau_out);
/* 单关节的控制量边界，= |Δτ|<=DTAU_MAX 和 |tau_ff+Δτ|<=EFFORT_MAX 的交集。
 * mpc_cs_control_bounds() 内部用它；暴露出来便于上层做诊断。 */
void mpc_control_bounds(int j, float tau_ff_j, float *lo, float *hi);

/* 装配总力矩并按 EFFORT_MAX 饱和：tau_out[j] = clamp(tau_ff[j]+dtau[j])。
 * 返回被截断的关节数。
 *
 * **必须调这个，而不是自己写 tau_ff+dtau。** 现在每拍都解全部关节，
 * 求解器的控制量箱约束已经保证 |tau_ff+Δτ| <= EFFORT_MAX，这里是第二道保险：
 * 防的是调用方把 Δτ 和别的 tau_ff 拼在一起、或者跳过某一拍的求解（hold）。 */
int mpc_saturate_total(const float *tau_ff, const float *dtau, float *tau_out);

/* 关节电机转子惯量（折算到输出端, kg*m^2），已计入 rnea / crba:
 * armature = [0.03058, 0.13689, 0.13472, 0.00175, 0.03089, 0.0023 ] */

/* ================= P3: 惯量随构型缩放 =================
 * QP 的决策变量 u 是"按设计惯量 MPC_M_DESIGN 换算的力矩"，
 * 真正下发的修正量 Δτ = ρ(q)·u，ρ = M_eff(q)/MPC_M_DESIGN，
 * M_eff = 1/(M(q)^-1)_jj。这样闭环对 u 来说是线性定常的，QP 矩阵不用变。
 * MPC_USE_RHO = 0 时 ρ 恒为 1（旧行为）。 */
#define MPC_USE_RHO  1
extern const float MPC_M_DESIGN[MPC_NJ];
extern const float MPC_RHO_MIN[MPC_NJ];
extern const float MPC_RHO_MAX[MPC_NJ];
/* 每拍调一次。q 是测量到的关节角。内部：crba() 算 M(q)，6x6 Cholesky
 * 求 (M^-1)_jj，再钳到 [RHO_MIN, RHO_MAX]（高估惯量 = 过增益，必须钳）。 */
void mpc_inertia_ratio(const float q[MPC_NJ], float rho[MPC_NJ]);

/* ================= P1 + P2: 观测器 + 延迟补偿 =================
 * 每个关节一个 3 状态观测器 z = [e, de, w]，w 是模型没解释的加速度。
 * MPC_EST_D = 反馈延迟 + 执行延迟（拍），观测器在测量时刻更新，
 * 再用已发出去的 D 个 u 前推到力矩生效时刻。
 *
 * 每个 tick 的调用顺序（和 sim_core.run() 逐行对应，全部关节每拍都解）：
 *   0. mpc_inertia_ratio(q_meas, rho)；tau_ff = rnea(...) + mpc_friction_ff(dq_meas)
 *      tau_ff_end（见 mpc_cs_step 的说明，要多算两次 RNEA，六个关节共用）
 *   1. 所有关节  mpc_est_update(j, e_meas, de_meas)
 *        e_meas = q_ref(t_meas) - q_meas，t_meas = 现在 - MPC_EST_FB 拍
 *   2. 所有关节  mpc_est_anchor(j, anchor, &w_hat) -> u[j] = mpc_cs_step(...)
 *   3. 所有关节  Δτ = ρ·u，mpc_saturate_total() 饱和
 *   4. 所有关节  mpc_est_commit(j, u_applied)，u_applied = (tau - tau_ff)/ρ
 *        必须用饱和**之后**反算的值：观测器只认真正发出去的量，
 *        所以积分作用天然不会饱和累积。 */
#define MPC_EST_D    2
#define MPC_EST_FB   1
#define MPC_EST_CT   1
extern const float MPC_OBS_F[MPC_NJ][9];   /* 行主序 3x3 */
extern const float MPC_OBS_G[MPC_NJ][3];
extern const float MPC_OBS_L[MPC_NJ][6];   /* 行主序 3x2 */
/* 逐关节：ŵ 是否进 MPC 的预测模型（积分作用）。0 的关节观测器只做滤波和延迟补偿。 */
extern const unsigned char MPC_USE_WCOMP[MPC_NJ];
void  mpc_est_reset(void);
void  mpc_est_set(int j, const float z[3], const float hist[MPC_EST_D + 1]);
void  mpc_est_update(int j, float e_meas, float de_meas);
void  mpc_est_anchor(int j, float anchor[2], float *w_hat);
void  mpc_est_commit(int j, float u_applied);
float mpc_est_w(int j);

/* 锚点误差钳位上限（factor=3），由新求解器自己的无约束增益反推：
 * K_pos = [165.27, 292.22, 291.52,  27.2 , 158.5 ,  14.86]
 * K_vel = [ 6.02, 16.57, 14.12,  0.95,  5.75,  0.53]
 * 只钳进代价的锚点，不钳进约束预测的锚点（约束要按真实状态预测位置）。
 * mpc_cs_step() 内部已自动钳位，调用方无需自己处理。 */
extern const float MPC_E_MAX[MPC_NJ];
extern const float MPC_DE_MAX[MPC_NJ];
/* 把锚点误差钳到 [-MPC_E_MAX, MPC_E_MAX] x [-MPC_DE_MAX, MPC_DE_MAX]。
 * out 可以和 in 指向同一块内存。 */
void mpc_clamp_anchor(int j, const float in[2], float out[2]);

/* 每块控制量 u 的硬约束（和 Python cmpc.control_bounds_blocks() 逐行对应）。
 * 预测时域里的前馈按 tau_ff0 -> tau_ff_end 线性插值，每块取块内首尾两步
 * 边界的交集，再除以 ρ（Δτ = ρ·u）。 */
void mpc_cs_control_bounds(int j, float tau_ff0, float tau_ff_end, float rho_j,
                           float lo[MPC_CS_NU], float hi[MPC_CS_NU]);
/* 每个检查点的误差状态 [e, de] 的软约束（和 cmpc.state_bounds() 对应），
 * 来自绝对限位 MPC_Q_LO/HI、MPC_V_HI 和该时刻的参考。交错存放 [pos, vel]。 */
void mpc_cs_state_bounds(int j, const float qr_e[MPC_CS_NB],
                         const float dqr_e[MPC_CS_NB],
                         float lo[2 * MPC_CS_NB], float hi[2 * MPC_CS_NB]);
/* 定迭代 ADMM 一次（和 cmpc.Solver.solve() 逐行对应），返回首个控制量 u0，
 * 一定在 [lo_u[0], hi_u[0]] 内。热启动状态在内部按关节保存。
 *   w0_cost  进代价的锚点（钳过位）    w0_true  进约束预测的锚点（没钳位）
 *   w_hat    扰动加速度（不用扰动补偿的关节传 0） */
float mpc_cs_solve(int j, const float w0_cost[2], const float w0_true[2],
                   float w_hat, const float lo_u[MPC_CS_NU],
                   const float hi_u[MPC_CS_NU], const float lo_s[2 * MPC_CS_NB],
                   const float hi_s[2 * MPC_CS_NB]);

/* 一个关节一拍的完整求解：钳位 + ŵ 掩码 + 两组边界 + mpc_cs_solve()。
 * 返回 u（不是 Δτ：Δτ = rho_j * u）。
 *   anchor     mpc_est_anchor() 给的锚点 [e, de]（力矩生效时刻）
 *   w_hat      mpc_est_anchor() 给的 ŵ，MPC_USE_WCOMP[j]=0 时内部置 0
 *   tau_ff0    本拍该关节的前馈（RNEA(q_meas, dq_meas, ddq_ref(t_apply)) + 摩擦）
 *   tau_ff_end 时域末尾的前馈预测 = tau_ff0 + (tref(t_apply+NS*DT) - tref(t_apply))，
 *              tref = RNEA(参考) + 摩擦(参考速度)
 *   qr_e/dqr_e 每个检查点的参考，时刻见 MPC_CS_ENDS */
float mpc_cs_step(int j, const float anchor[2], float w_hat, float tau_ff0,
                  float tau_ff_end, float rho_j, const float qr_e[MPC_CS_NB],
                  const float dqr_e[MPC_CS_NB]);

/* 热启动状态。上电、急停恢复、切换控制模式时调 mpc_cs_reset()。
 * mpc_cs_set_warm() 只给回归测试用（从 Python 拷状态）。 */
void mpc_cs_reset(void);
void mpc_cs_set_warm(int j, const float u[MPC_CS_NU], const float zu[MPC_CS_NU],
                     const float yu[MPC_CS_NU], const float zs[2 * MPC_CS_NB],
                     const float ys[2 * MPC_CS_NB]);

#ifdef __cplusplus
}
#endif
#endif
