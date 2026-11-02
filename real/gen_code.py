# gen_code.py
# 生成 STM32 可用的 C 代码
import warnings
warnings.filterwarnings("ignore")

import os
import shutil
from pathlib import Path
import numpy as np
import casadi as ca
import pinocchio as pin
import pinocchio.casadi as cpin
import mpc_config as cfg
import dyn_model as dyn
import cmpc

# ================== 配置 ==================
URDF_PATH   = cfg.URDF_PATH
SCRIPT_DIR  = Path(__file__).resolve().parent
OUT_DIR     = SCRIPT_DIR / "generated"

BASE_DT     = cfg.BASE_DT
CASADI_REAL = "float"
# ==========================================

os.makedirs(OUT_DIR, exist_ok=True)

# ---------- 1. 加载 URDF ----------
# armature（折算到输出端的转子惯量）必须在建 casadi 模型之前设好：
# pinocchio 的 rnea / crba / aba 都会计入它，cpin.Model 拿的是设好之后的快照，
# 所以生成的 rnea_6axis.c 和残差模型的惯量来自同一套参数。
model  = pin.buildModelFromUrdf(URDF_PATH)
n = model.nv
assert model.nq == n, "只支持固定基、全旋转关节"
if n != 6:
    raise ValueError(f"current generator requires a 6-axis URDF, got {n} velocity DOFs")
armature = np.asarray(cfg.ARMATURE, dtype=float)
if armature.size != n:
    raise ValueError(f"ARMATURE 长度必须等于关节数 {n}，got {armature.size}")
if np.any(armature < 0.0) or not np.all(np.isfinite(armature)):
    raise ValueError(f"ARMATURE 必须是非负有限值，got {armature}")
model.armature = armature
data   = model.createData()
cmodel = cpin.Model(model)
cdata  = cmodel.createData()

dtau_max = np.asarray(cfg.DTAU_MAX, dtype=float)
if dtau_max.ndim == 0:
    dtau_max = np.full(n, float(dtau_max))
if dtau_max.size != n:
    raise ValueError(f"DTAU_MAX 长度必须等于关节数 {n}，got {dtau_max.size}")
if np.any(dtau_max <= 0.0) or not np.all(np.isfinite(dtau_max)):
    raise ValueError(f"DTAU_MAX 必须是正有限值，got {dtau_max}")

# 关节力矩绝对上限：QP 里约束 |tau_ff + Δτ| <= effort。
# 和仿真调同一个函数（dyn.effort_array），保证两边的边界逐位一致。
effort = dyn.effort_array(cfg, model)
fc = np.asarray(cfg.FRICTION_COULOMB, dtype=float)
fv = np.asarray(cfg.FRICTION_VISCOUS, dtype=float)
for name, arr in (("FRICTION_COULOMB", fc), ("FRICTION_VISCOUS", fv)):
    if arr.size != n or np.any(arr < 0.0) or not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} 必须是长度 {n} 的非负有限值，got {arr}")

print(f"[1/4] URDF OK, n = {n}")
print(f"     URDF      = {URDF_PATH}")
print(f"     armature  = {armature}")
print(f"     effort    = {effort}")
print(f"     DTAU_MAX  = {dtau_max}")
print(f"     摩擦前馈   = {cfg.FRICTION_FF_SCALE:g} * (fc*tanh(dq/{cfg.FRICTION_FF_DQ_EPS:g}) + fv*dq)")

# ---------- 2. 设计惯量 ----------
# 残差模型的惯量：用等效惯量 1/(M^-1)_jj，不是 M_jj。理由见 dyn_model.py。
# 必须和 sim.py 用同一个 INERTIA_MODE/SAMPLES/SEED，否则仿真和上板行为不一致。
M_design, inertia_info = dyn.design_inertia(
    model, data, cfg.INERTIA_MODE,
    n_samples=cfg.INERTIA_SAMPLES, seed=cfg.INERTIA_SEED,
    extra_poses=(np.array(cfg.Q_GOAL, dtype=float),))
dyn.print_inertia_report(inertia_info)

if not np.all(np.isfinite(M_design)) or np.any(M_design <= 0.0):
    raise ValueError(f"design inertia must be finite and positive, got {M_design}")

# ---------- 3. 生成 rnea_6axis.c/h ----------
q_sym   = ca.SX.sym("q",   n)
dq_sym  = ca.SX.sym("dq",  n)
ddq_sym = ca.SX.sym("ddq", n)
tau_sym = cpin.rnea(cmodel, cdata, q_sym, dq_sym, ddq_sym)
f_rnea  = ca.Function("rnea", [q_sym, dq_sym, ddq_sym], [tau_sym])

# with_header 必须显式打开，否则 CasADi 只生成 .c，调用方拿不到 rnea() 的声明。
cg = ca.CodeGenerator("rnea_6axis",
                      {"casadi_real": CASADI_REAL, "with_header": True})
cg.add(f_rnea)
cwd = os.getcwd(); os.chdir(OUT_DIR)
try:
    cg.generate()
finally:
    os.chdir(cwd)
print("[2/4] rnea_6axis.c/h 生成")

# ---------- 3b. 生成 crba_6axis.c/h（P3 的惯量矩阵）----------
# pinocchio 的 crba 只填上三角，这里补成完整对称矩阵再输出，C 侧直接按
# 列主序读 36 个数。它和 rnea 用同一个 cmodel，所以 armature 也自动计入。
M_sym = cpin.crba(cmodel, cdata, q_sym)
M_sym = ca.triu(M_sym) + ca.triu(M_sym, False).T
f_crba = ca.Function("crba", [q_sym], [M_sym])
cg2 = ca.CodeGenerator("crba_6axis",
                       {"casadi_real": CASADI_REAL, "with_header": True})
cg2.add(f_crba)
cwd = os.getcwd(); os.chdir(OUT_DIR)
try:
    cg2.generate()
finally:
    os.chdir(cwd)
print("     crba_6axis.c/h 生成（P3 用）")

# ---------- 3c. P1/P2/P3 的常数 ----------
# 和 sim_core.run() 调同一组函数，保证板上和仿真用的是同一套数。
if cfg.MPC_FORM not in ("ddq", "tau"):
    raise ValueError(f"MPC_FORM 只能是 'ddq' 或 'tau'，got {cfg.MPC_FORM!r}")
if cfg.ESTIMATOR != "kalman":
    raise ValueError("gen_code 只支持 ESTIMATOR='kalman'；'none' 只用于仿真对照")
USE_RHO = cfg.MPC_FORM == "ddq"
rho_lo, rho_hi = dyn.inertia_ratio_limits(cfg, inertia_info)
obs_F, obs_G, obs_L = dyn.observer_design(cfg, M_design)
EST_FB = int(cfg.EST_FB_DELAY_TICKS) if cfg.DELAY_COMP else 0
EST_CT = int(cfg.EST_CTRL_DELAY_TICKS) if cfg.DELAY_COMP else 0
EST_D = EST_FB + EST_CT
wcomp = dyn.dist_comp_mask(cfg, n)
print(f"     MPC_FORM  = {cfg.MPC_FORM}   ρ 范围 [{rho_lo.min():.2f}, {rho_hi}]")
print(f"     观测器    = Kalman，延迟补偿 D = {EST_FB} + {EST_CT} = {EST_D} 拍")
print(f"     ŵ 进 MPC  = {wcomp.astype(int)}")

# ---------- 4. P4/P5/P7: 消元 + 定迭代 ADMM 的常数 ----------
# 和 sim_core.run() 调同一个 cmpc.design()，保证板上和仿真是同一套矩阵。
if cfg.SOLVER != "cs_admm":
    raise ValueError("gen_code 只生成 SOLVER='cs_admm'；'osqp' 只留在仿真里做对照")
cs = cmpc.design(cfg, M_design)
NU, NB, NS, ITERS = cs["NU"], cs["NB"], cs["NS"], cs["iters"]
if cs["blocks"][0] != 1:
    raise ValueError("MPC_BLOCKS 第一块必须是 1 拍（每拍重解，发出去的就是第一块）")
E_MAX, DE_MAX = cmpc.clamp_limits(cfg, cs, dtau_max)
CLAMP_ON = E_MAX is not None
q_lo, q_hi, v_hi = cmpc.joint_limits(cfg, model)
# 软约束 prox 的收缩系数只和常数有关，离线算好
cs_ks = cs["rho_s"][:, None] / (cs["rho_s"][:, None] + cs["wsoft"])
print(f"[3/4] 消元 ADMM 常数: NU={NU}, 约束检查点 NB={NB}, 时域 {NS} 拍 = {NS*BASE_DT*1e3:g} ms, "
      f"定迭代 {ITERS} 次")
print(f"     blocks = {cs['blocks'].tolist()}")
print(f"     H 条件数 = {np.array2string(cs['cond'], precision=1, separator=', ')}")
print(f"     K_pos  = {np.array2string(cs['K0'][:, 0], precision=2, separator=', ')}")
print(f"     K_vel  = {np.array2string(cs['K0'][:, 1], precision=2, separator=', ')}")
if CLAMP_ON:
    if not (np.all(np.isfinite(E_MAX)) and np.all(E_MAX > 0.0)
            and np.all(np.isfinite(DE_MAX)) and np.all(DE_MAX > 0.0)):
        raise ValueError(f"clamp limits must be finite and positive, got {E_MAX}, {DE_MAX}")
    print(f"     E_MAX  = {E_MAX}   (factor={cfg.E_CLAMP_FACTOR:g})")
    print(f"     DE_MAX = {DE_MAX}")
print(f"     限位   q ∈ [{q_lo}, {q_hi}]")
print(f"            |dq| <= {v_hi}")

# 旧版的 OSQP workspace 目录是本脚本生成的，现在不再需要，删掉以免误编译。
old_solver_dir = OUT_DIR / "generated_solver"
if old_solver_dir.exists():
    shutil.rmtree(old_solver_dir)
with open(OUT_DIR / "sources.txt", "w", encoding="utf-8", newline="\n") as manifest:
    manifest.write("\n".join([
        "# 需要编译的文件（都在 generated/ 下），不再依赖 OSQP：",
        "mpc_6axis.c",
        "rnea_6axis.c",
        "crba_6axis.c",
        "# include 目录：-I generated",
        "# 链接 libm（sqrtf、tanhf）",
    ]) + "\n")

dtau_max_str = ", ".join(f"{v:.8e}f" for v in dtau_max)
effort_str = ", ".join(f"{v:.8e}f" for v in effort)
fc_str = ", ".join(f"{v:.8e}f" for v in fc)
fv_str = ", ".join(f"{v:.8e}f" for v in fv)
armature_str = np.array2string(armature, precision=8, separator=', ')
_f8 = lambda arr: ", ".join(f"{v:.9e}f" for v in np.asarray(arr).ravel())
m_design_str = _f8(M_design)
rho_lo_str = _f8(rho_lo)
rho_hi_str = _f8(rho_hi)
# 注意这三行是普通字符串，不是 f-string：花括号写一层。
# 写成 {{ }} 会原样进到 C 里变成双层花括号，编译器当成标量初始化，
# 矩阵内容就错了（只报 warning，不报 error）。
obs_f_str = ",\n".join("    { " + _f8(obs_F[j]) + " }" for j in range(n))
obs_g_str = ",\n".join("    { " + _f8(obs_G[j]) + " }" for j in range(n))
obs_l_str = ",\n".join("    { " + _f8(obs_L[j]) + " }" for j in range(n))
wcomp_str = ", ".join(str(int(v)) for v in wcomp)

# 钳位关闭时上限填 +inf，C 侧的 clamp 代码就退化成恒等变换，
# 两条分支共用一份实现，避免仿真和 C 出现结构性差异。
_e_max_vals  = E_MAX  if CLAMP_ON else np.full(n, np.inf)
_de_max_vals = DE_MAX if CLAMP_ON else np.full(n, np.inf)
_fmt = lambda v: "INFINITY" if not np.isfinite(v) else f"{v:.8e}f"
e_max_str  = ", ".join(_fmt(v) for v in _e_max_vals)
de_max_str = ", ".join(_fmt(v) for v in _de_max_vals)
clamp_note = (f"factor={cfg.E_CLAMP_FACTOR:g}" if CLAMP_ON else "关闭 (INFINITY)")
# P7 常数表。和观测器那三行一样是普通字符串，花括号只写一层。
def _tab2(arr):
    """(n, m) -> 每个关节一行的 C 初始化式"""
    return ",\n".join("    { " + _f8(arr[j]) + " }" for j in range(n))
cs_kinv_str = _tab2(cs["Kinv"].reshape(n, -1))   # 行主序 NU x NU
cs_c_str    = _tab2(cs["C"].reshape(n, -1))      # 行主序 2NB x NU
cs_rs_str   = _tab2(cs["rs"])
cs_gw_str   = _tab2(cs["Gw"].reshape(n, -1))     # 行主序 NU x 2
cs_gc_str   = _tab2(cs["Gc"])
cs_pw_str   = _tab2(cs["Pw"].reshape(n, -1))     # 行主序 2NB x 2
cs_pc_str   = _tab2(cs["Pc"])
cs_ks_str   = _tab2(cs_ks)
cs_rho_u_str = _f8(cs["rho_u"])
cs_rho_s_str = _f8(cs["rho_s"])
cs_blocks_str = ", ".join(str(int(v)) for v in cs["blocks"])
cs_ends_str = ", ".join(str(int(v)) for v in cs["ends"])
q_lo_str = _f8(q_lo)
q_hi_str = _f8(q_hi)
v_hi_str = _f8(v_hi)
K0_note = (f"K_pos = {np.array2string(cs['K0'][:, 0], precision=2, separator=', ')}\n"
           f" * K_vel = {np.array2string(cs['K0'][:, 1], precision=2, separator=', ')}")

header = f"""#ifndef MPC_6AXIS_H
#define MPC_6AXIS_H
#ifdef __cplusplus
extern "C" {{
#endif

#define MPC_NJ  {n}
/* 每拍（MPC_BASE_DT）解全部 MPC_NJ 个关节，预测步长也是 MPC_BASE_DT（P4）。 */
#define MPC_BASE_DT  {BASE_DT}f

/* ================= P4/P5/P7: 消元 + 定迭代 ADMM =================
 * 预测时域 MPC_CS_NS 拍，分成 MPC_CS_NU 块（块内控制量不变），
 * 每块末尾检查一次位置/速度约束，共 MPC_CS_NB 个检查点。
 * 每次求解固定迭代 MPC_CS_ITERS 次，耗时是常数。 */
#define MPC_CS_NU     {NU}
#define MPC_CS_NB     {NB}
#define MPC_CS_NS     {NS}
#define MPC_CS_ITERS  {ITERS}
/* 块长和块末尾的步号（1 起）。调用方要按 MPC_CS_ENDS 取参考预览：
 * 第 i 个检查点的参考时刻 = 力矩生效时刻 + MPC_CS_ENDS[i] * MPC_BASE_DT。 */
extern const unsigned char MPC_CS_BLOCKS[MPC_CS_NU];
extern const unsigned char MPC_CS_ENDS[MPC_CS_NB];
#define MPC_CS_ENDS_LIST {{ {cs_ends_str} }}

/* 关节限位（已扣余量 POS/VEL_LIMIT_MARGIN）。q ∈ [Q_LO, Q_HI]，|dq| <= V_HI。 */
extern const float MPC_Q_LO[MPC_NJ];
extern const float MPC_Q_HI[MPC_NJ];
extern const float MPC_V_HI[MPC_NJ];

/* 修正量上限 [Nm]，逐关节。各关节的等效惯量相差三个数量级，
 * 共用一个标量会让手腕的钳位上限失去意义，所以这里是数组。
 * MPC_DTAU_MAX_WORST 是六轴里的最大值，只用于给调用方开数组、做量级断言。 */
extern const float MPC_DTAU_MAX[MPC_NJ];
#define MPC_DTAU_MAX_WORST  {float(dtau_max.max()):.8e}f
/* 花括号初始化式，给只包含本头文件、不链接 mpc_6axis.c 的消费者
 * （例如 ctest/test_rnea）用：static const float lim[MPC_NJ] = MPC_DTAU_MAX_LIST; */
#define MPC_DTAU_MAX_LIST {{ {dtau_max_str} }}

/* 关节力矩绝对上限 [Nm]（电机能出的力，来自 URDF 的 effort）。
 * QP 把 |tau_ff + Δτ| 约束在这个值以内——不是只约束 Δτ。
 * mpc_cs_step() 需要当前和时域末尾的 tau_ff 才能算出这个边界。 */
extern const float MPC_EFFORT_MAX[MPC_NJ];
#define MPC_EFFORT_MAX_LIST {{ {effort_str} }}

/* 摩擦前馈补偿的系数。板上必须用这一组，否则总力矩约束里的 tau_ff
 * 和实际下发的不是一个东西。 */
extern const float MPC_FRIC_FC[MPC_NJ];
extern const float MPC_FRIC_FV[MPC_NJ];
#define MPC_FRIC_SCALE  {cfg.FRICTION_FF_SCALE:.8e}f
#define MPC_FRIC_DQ_EPS {cfg.FRICTION_FF_DQ_EPS:.8e}f

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
 * armature = {armature_str} */

/* ================= P3: 惯量随构型缩放 =================
 * QP 的决策变量 u 是"按设计惯量 MPC_M_DESIGN 换算的力矩"，
 * 真正下发的修正量 Δτ = ρ(q)·u，ρ = M_eff(q)/MPC_M_DESIGN，
 * M_eff = 1/(M(q)^-1)_jj。这样闭环对 u 来说是线性定常的，QP 矩阵不用变。
 * MPC_USE_RHO = 0 时 ρ 恒为 1（旧行为）。 */
#define MPC_USE_RHO  {1 if USE_RHO else 0}
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
#define MPC_EST_D    {EST_D}
#define MPC_EST_FB   {EST_FB}
#define MPC_EST_CT   {EST_CT}
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

/* 锚点误差钳位上限（{clamp_note}），由新求解器自己的无约束增益反推：
 * {K0_note}
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
}}
#endif
#endif
"""

source = f"""#include <math.h>

#include "mpc_6axis.h"
#include "crba_6axis.h"

/* 必须定义在所有使用者之前。放在后面时 C99 以前的编译器会按隐式声明
 * 处理成返回 int——力矩被截成整数，而 gcc 14 起直接报错。 */
static float mpc_clamp1(float v, float lim)
{{
    if (v >  lim) return  lim;
    if (v < -lim) return -lim;
    return v;
}}

/* 逐关节修正量上限。 */
const float MPC_DTAU_MAX[MPC_NJ] = {{ {dtau_max_str} }};
/* 关节力矩绝对上限，总力矩约束用。 */
const float MPC_EFFORT_MAX[MPC_NJ] = {{ {effort_str} }};
/* 摩擦前馈系数。 */
const float MPC_FRIC_FC[MPC_NJ] = {{ {fc_str} }};
const float MPC_FRIC_FV[MPC_NJ] = {{ {fv_str} }};

/* ================= P3: 惯量比 ================= */
const float MPC_M_DESIGN[MPC_NJ] = {{ {m_design_str} }};
const float MPC_RHO_MIN[MPC_NJ] = {{ {rho_lo_str} }};
const float MPC_RHO_MAX[MPC_NJ] = {{ {rho_hi_str} }};

#if MPC_USE_RHO
/* crba() 是 CasADi 生成的，输出完整对称的 M(q)，列主序 36 个数。 */
#if crba_SZ_IW > 0
static casadi_int mpc_crba_iw[crba_SZ_IW];
#define MPC_CRBA_IW mpc_crba_iw
#else
#define MPC_CRBA_IW 0
#endif
#if crba_SZ_W > 0
static casadi_real mpc_crba_w[crba_SZ_W];
#define MPC_CRBA_W mpc_crba_w
#else
#define MPC_CRBA_W 0
#endif
#endif

void mpc_inertia_ratio(const float q[MPC_NJ], float rho[MPC_NJ])
{{
    if (q == 0 || rho == 0) return;
#if MPC_USE_RHO
    casadi_real cq[MPC_NJ], Mcol[MPC_NJ * MPC_NJ];
    for (int j = 0; j < MPC_NJ; ++j) cq[j] = (casadi_real)q[j];
    const casadi_real *arg[1] = {{ cq }};
    casadi_real *res[1] = {{ Mcol }};
    if (crba(arg, res, MPC_CRBA_IW, MPC_CRBA_W, 0) != 0) {{
        for (int j = 0; j < MPC_NJ; ++j) rho[j] = MPC_RHO_MIN[j];
        return;
    }}
    /* Cholesky M = L L^T，X = L^-1，(M^-1)_jj = sum_i X_ij^2。
     * 和 Python 侧 dyn_model.minv_diag_chol() 逐行对应。 */
    float L[MPC_NJ][MPC_NJ], X[MPC_NJ][MPC_NJ];
    for (int i = 0; i < MPC_NJ; ++i)
        for (int k = 0; k < MPC_NJ; ++k) {{ L[i][k] = 0.0f; X[i][k] = 0.0f; }}
    for (int i = 0; i < MPC_NJ; ++i) {{
        for (int k = 0; k <= i; ++k) {{
            float acc = (float)Mcol[k * MPC_NJ + i];   /* M[i][k]，列主序 */
            for (int p = 0; p < k; ++p) acc -= L[i][p] * L[k][p];
            if (i == k) {{
                if (!(acc > 0.0f)) {{             /* 不正定：退到最保守的值 */
                    for (int j = 0; j < MPC_NJ; ++j) rho[j] = MPC_RHO_MIN[j];
                    return;
                }}
                L[i][i] = sqrtf(acc);
            }} else {{
                L[i][k] = acc / L[k][k];
            }}
        }}
    }}
    for (int i = 0; i < MPC_NJ; ++i) {{
        X[i][i] = 1.0f / L[i][i];
        for (int k = 0; k < i; ++k) {{
            float acc = 0.0f;
            for (int p = k; p < i; ++p) acc += L[i][p] * X[p][k];
            X[i][k] = -acc / L[i][i];
        }}
    }}
    for (int j = 0; j < MPC_NJ; ++j) {{
        float s = 0.0f;
        for (int i = 0; i < MPC_NJ; ++i) s += X[i][j] * X[i][j];
        float r = (1.0f / s) / MPC_M_DESIGN[j];
        if (r < MPC_RHO_MIN[j]) r = MPC_RHO_MIN[j];
        if (r > MPC_RHO_MAX[j]) r = MPC_RHO_MAX[j];
        rho[j] = r;
    }}
#else
    for (int j = 0; j < MPC_NJ; ++j) rho[j] = 1.0f;
#endif
}}

/* ================= P1 + P2: 观测器 + 延迟补偿 ================= */
const float MPC_OBS_F[MPC_NJ][9] = {{
{obs_f_str}
}};
const float MPC_OBS_G[MPC_NJ][3] = {{
{obs_g_str}
}};
const float MPC_OBS_L[MPC_NJ][6] = {{
{obs_l_str}
}};
const unsigned char MPC_USE_WCOMP[MPC_NJ] = {{ {wcomp_str} }};

/* 观测器状态。只被控制任务读写，不用加锁。 */
static float mpc_est_z[MPC_NJ][3];
/* 移位寄存器：hist[j][0] = u[k-1-D]，hist[j][1..D] = u[k-D..k-1] */
static float mpc_est_hist[MPC_NJ][MPC_EST_D + 1];

/* z⁺ = F z + G u，3x3 行主序 */
static void mpc_est_step(int j, const float z[3], float u, float out[3])
{{
    const float *F = MPC_OBS_F[j];
    const float *G = MPC_OBS_G[j];
    for (int r = 0; r < 3; ++r)
        out[r] = F[3*r]*z[0] + F[3*r+1]*z[1] + F[3*r+2]*z[2] + G[r]*u;
}}

void mpc_est_reset(void)
{{
    for (int j = 0; j < MPC_NJ; ++j) {{
        for (int r = 0; r < 3; ++r) mpc_est_z[j][r] = 0.0f;
        for (int i = 0; i <= MPC_EST_D; ++i) mpc_est_hist[j][i] = 0.0f;
    }}
}}

void mpc_est_set(int j, const float z[3], const float hist[MPC_EST_D + 1])
{{
    if (j < 0 || j >= MPC_NJ || z == 0 || hist == 0) return;
    for (int r = 0; r < 3; ++r) mpc_est_z[j][r] = z[r];
    for (int i = 0; i <= MPC_EST_D; ++i) mpc_est_hist[j][i] = hist[i];
}}

void mpc_est_update(int j, float e_meas, float de_meas)
{{
    if (j < 0 || j >= MPC_NJ) return;
    float zp[3];
    mpc_est_step(j, mpc_est_z[j], mpc_est_hist[j][0], zp);
    const float i0 = e_meas - zp[0], i1 = de_meas - zp[1];
    const float *L = MPC_OBS_L[j];
    for (int r = 0; r < 3; ++r)
        mpc_est_z[j][r] = zp[r] + L[2*r]*i0 + L[2*r+1]*i1;
}}

void mpc_est_anchor(int j, float anchor[2], float *w_hat)
{{
    if (j < 0 || j >= MPC_NJ || anchor == 0) return;
    float z[3] = {{ mpc_est_z[j][0], mpc_est_z[j][1], mpc_est_z[j][2] }};
    for (int i = 1; i <= MPC_EST_D; ++i) {{
        float zn[3];
        mpc_est_step(j, z, mpc_est_hist[j][i], zn);
        z[0] = zn[0]; z[1] = zn[1]; z[2] = zn[2];
    }}
    anchor[0] = z[0];
    anchor[1] = z[1];
    if (w_hat != 0) *w_hat = z[2];
}}

void mpc_est_commit(int j, float u_applied)
{{
    if (j < 0 || j >= MPC_NJ) return;
    for (int i = 0; i < MPC_EST_D; ++i) mpc_est_hist[j][i] = mpc_est_hist[j][i + 1];
    mpc_est_hist[j][MPC_EST_D] = u_applied;
}}

float mpc_est_w(int j)
{{
    if (j < 0 || j >= MPC_NJ) return 0.0f;
    return mpc_est_z[j][2];
}}

void mpc_friction_ff(const float *dq, float *tau_out)
{{
    if (dq == 0 || tau_out == 0) return;
    for (int j = 0; j < MPC_NJ; ++j) {{
        tau_out[j] = MPC_FRIC_SCALE *
            (MPC_FRIC_FC[j] * tanhf(dq[j] / MPC_FRIC_DQ_EPS)
             + MPC_FRIC_FV[j] * dq[j]);
    }}
}}

/* |Δτ| <= DTAU_MAX 和 |tau_ff + Δτ| <= EFFORT_MAX 的交集。
 * tau_ff 已经超过 EFFORT_MAX + DTAU_MAX 时交集为空，这时把区间塌成单点
 * （朝把总力矩拉回限内的方向顶满），而不是交给 OSQP 一个不可行问题。
 * 和 Python 侧 dyn_model.control_bounds() 逐行对应。 */
void mpc_control_bounds(int j, float tau_ff_j, float *lo, float *hi)
{{
    if (j < 0 || j >= MPC_NJ || lo == 0 || hi == 0) return;
    const float dmax = MPC_DTAU_MAX[j];
    const float emax = MPC_EFFORT_MAX[j];
    float a = -dmax, b = dmax;
    const float ea = -emax - tau_ff_j, eb = emax - tau_ff_j;
    if (ea > a) a = ea;
    if (eb < b) b = eb;
    if (a > b) {{
        const float v = (tau_ff_j > 0.0f) ? -dmax : dmax;
        *lo = v; *hi = v;
        return;
    }}
    *lo = a; *hi = b;
}}

int mpc_saturate_total(const float *tau_ff, const float *dtau, float *tau_out)
{{
    if (tau_ff == 0 || dtau == 0 || tau_out == 0) return 0;
    int n_sat = 0;
    for (int j = 0; j < MPC_NJ; ++j) {{
        const float raw = tau_ff[j] + dtau[j];
        const float v = mpc_clamp1(raw, MPC_EFFORT_MAX[j]);
        if (v != raw) n_sat++;
        tau_out[j] = v;
    }}
    return n_sat;
}}

/* 锚点误差钳位上限（{clamp_note}）。由新求解器的无约束增益反推：
 * {K0_note} */
const float MPC_E_MAX[MPC_NJ] = {{ {e_max_str} }};
const float MPC_DE_MAX[MPC_NJ] = {{ {de_max_str} }};

void mpc_clamp_anchor(int j, const float in[2], float out[2])
{{
    if (j < 0 || j >= MPC_NJ || in == 0 || out == 0) return;
    out[0] = mpc_clamp1(in[0], MPC_E_MAX[j]);
    out[1] = mpc_clamp1(in[1], MPC_DE_MAX[j]);
}}

/* ================= P4/P5/P7: 消元 + 定迭代 ADMM =================
 * 常数全部来自 cmpc.design()，离线算好（包括 K^-1）。
 * 在线每次迭代：K^-1 乘向量（NU x NU）、C 和 C^T 乘向量（2NB x NU）、逐元素投影。 */
#define MPC_CS_NM  (2 * MPC_CS_NB)
const unsigned char MPC_CS_BLOCKS[MPC_CS_NU] = {{ {cs_blocks_str} }};
const unsigned char MPC_CS_ENDS[MPC_CS_NB] = {{ {cs_ends_str} }};
const float MPC_Q_LO[MPC_NJ] = {{ {q_lo_str} }};
const float MPC_Q_HI[MPC_NJ] = {{ {q_hi_str} }};
const float MPC_V_HI[MPC_NJ] = {{ {v_hi_str} }};

static const float MPC_CS_KINV[MPC_NJ][MPC_CS_NU * MPC_CS_NU] = {{
{cs_kinv_str}
}};
static const float MPC_CS_C[MPC_NJ][MPC_CS_NM * MPC_CS_NU] = {{
{cs_c_str}
}};
static const float MPC_CS_RS[MPC_NJ][MPC_CS_NM] = {{
{cs_rs_str}
}};
static const float MPC_CS_GW[MPC_NJ][MPC_CS_NU * 2] = {{
{cs_gw_str}
}};
static const float MPC_CS_GC[MPC_NJ][MPC_CS_NU] = {{
{cs_gc_str}
}};
static const float MPC_CS_PW[MPC_NJ][MPC_CS_NM * 2] = {{
{cs_pw_str}
}};
static const float MPC_CS_PC[MPC_NJ][MPC_CS_NM] = {{
{cs_pc_str}
}};
/* 软约束 prox 的收缩系数 rho_s / (rho_s + wsoft) */
static const float MPC_CS_KS[MPC_NJ][MPC_CS_NM] = {{
{cs_ks_str}
}};
static const float MPC_CS_RHO_U[MPC_NJ] = {{ {cs_rho_u_str} }};
static const float MPC_CS_RHO_S[MPC_NJ] = {{ {cs_rho_s_str} }};
#define MPC_CS_SIGMA  {cs["sigma"]:.9e}f
#define MPC_CS_ALPHA  {cs["alpha"]:.9e}f

/* 热启动状态，跨拍保留。只被控制任务读写，不用加锁。 */
static float mpc_cs_u[MPC_NJ][MPC_CS_NU];
static float mpc_cs_zu[MPC_NJ][MPC_CS_NU];
static float mpc_cs_yu[MPC_NJ][MPC_CS_NU];
static float mpc_cs_zs[MPC_NJ][MPC_CS_NM];
static float mpc_cs_ys[MPC_NJ][MPC_CS_NM];

static float mpc_clip(float v, float lo, float hi)
{{
    if (v < lo) v = lo;          /* 顺序和 np.clip 一致：先下界后上界 */
    if (v > hi) v = hi;
    return v;
}}

void mpc_cs_reset(void)
{{
    for (int j = 0; j < MPC_NJ; ++j) {{
        for (int i = 0; i < MPC_CS_NU; ++i)
            mpc_cs_u[j][i] = mpc_cs_zu[j][i] = mpc_cs_yu[j][i] = 0.0f;
        for (int r = 0; r < MPC_CS_NM; ++r)
            mpc_cs_zs[j][r] = mpc_cs_ys[j][r] = 0.0f;
    }}
}}

void mpc_cs_set_warm(int j, const float u[MPC_CS_NU], const float zu[MPC_CS_NU],
                     const float yu[MPC_CS_NU], const float zs[2 * MPC_CS_NB],
                     const float ys[2 * MPC_CS_NB])
{{
    if (j < 0 || j >= MPC_NJ || u == 0 || zu == 0 || yu == 0 || zs == 0 || ys == 0) return;
    for (int i = 0; i < MPC_CS_NU; ++i) {{
        mpc_cs_u[j][i] = u[i]; mpc_cs_zu[j][i] = zu[i]; mpc_cs_yu[j][i] = yu[i];
    }}
    for (int r = 0; r < MPC_CS_NM; ++r) {{
        mpc_cs_zs[j][r] = zs[r]; mpc_cs_ys[j][r] = ys[r];
    }}
}}

void mpc_cs_control_bounds(int j, float tau_ff0, float tau_ff_end, float rho_j,
                           float lo[MPC_CS_NU], float hi[MPC_CS_NU])
{{
    if (j < 0 || j >= MPC_NJ || lo == 0 || hi == 0) return;
    if (!(rho_j > 0.0f)) rho_j = 1.0f;
    int k0 = 0;
    for (int i = 0; i < MPC_CS_NU; ++i) {{
        const int L = MPC_CS_BLOCKS[i];
        const int kk[2] = {{ k0, k0 + L - 1 }};
        float a = -INFINITY, b = INFINITY, fa = 0.0f, fb = 0.0f;
        for (int m = 0; m < 2; ++m) {{
            const float tf = tau_ff0
                + ((float)kk[m] / (float)MPC_CS_NS) * (tau_ff_end - tau_ff0);
            float l_, h_;
            mpc_control_bounds(j, tf, &l_, &h_);
            if (m == 0) {{ fa = l_; fb = h_; }}
            if (l_ > a) a = l_;
            if (h_ < b) b = h_;
        }}
        /* 交集为空（前馈本身已经超限）：退回这块第一步的边界 */
        if (a > b) {{ a = fa; b = fb; }}
        lo[i] = a / rho_j;
        hi[i] = b / rho_j;
        k0 += L;
    }}
}}

void mpc_cs_state_bounds(int j, const float qr_e[MPC_CS_NB],
                         const float dqr_e[MPC_CS_NB],
                         float lo[2 * MPC_CS_NB], float hi[2 * MPC_CS_NB])
{{
    if (j < 0 || j >= MPC_NJ || qr_e == 0 || dqr_e == 0 || lo == 0 || hi == 0) return;
    for (int i = 0; i < MPC_CS_NB; ++i) {{
        lo[2*i]     = qr_e[i] - MPC_Q_HI[j];
        hi[2*i]     = qr_e[i] - MPC_Q_LO[j];
        lo[2*i + 1] = dqr_e[i] - MPC_V_HI[j];
        hi[2*i + 1] = dqr_e[i] + MPC_V_HI[j];
    }}
}}

float mpc_cs_solve(int j, const float w0_cost[2], const float w0_true[2],
                   float w_hat, const float lo_u[MPC_CS_NU],
                   const float hi_u[MPC_CS_NU], const float lo_s[2 * MPC_CS_NB],
                   const float hi_s[2 * MPC_CS_NB])
{{
    if (j < 0 || j >= MPC_NJ || w0_cost == 0 || w0_true == 0 ||
        lo_u == 0 || hi_u == 0 || lo_s == 0 || hi_s == 0) return 0.0f;
    const float *Kinv = MPC_CS_KINV[j];
    const float *C = MPC_CS_C[j];
    const float *Gw = MPC_CS_GW[j], *Pw = MPC_CS_PW[j];
    const float ru = MPC_CS_RHO_U[j], rss = MPC_CS_RHO_S[j];
    const float sg = MPC_CS_SIGMA, al = MPC_CS_ALPHA, al1 = 1.0f - MPC_CS_ALPHA;
    float f[MPC_CS_NU], ls[MPC_CS_NM], us[MPC_CS_NM];
    for (int i = 0; i < MPC_CS_NU; ++i)
        f[i] = Gw[2*i]*w0_cost[0] + Gw[2*i+1]*w0_cost[1] + MPC_CS_GC[j][i]*w_hat;
    for (int r = 0; r < MPC_CS_NM; ++r) {{
        const float off = Pw[2*r]*w0_true[0] + Pw[2*r+1]*w0_true[1]
                          + MPC_CS_PC[j][r]*w_hat;
        ls[r] = MPC_CS_RS[j][r] * (lo_s[r] - off);
        us[r] = MPC_CS_RS[j][r] * (hi_s[r] - off);
    }}
    float *u = mpc_cs_u[j], *zu = mpc_cs_zu[j], *yu = mpc_cs_yu[j];
    float *zs = mpc_cs_zs[j], *ys = mpc_cs_ys[j];
    const float *ks = MPC_CS_KS[j];
    float rhs[MPC_CS_NU], t[MPC_CS_NM];
    for (int it = 0; it < MPC_CS_ITERS; ++it) {{
        /* rhs = σu - f + ρu(zu - yu) + ρs Cᵀ(zs - ys) */
        for (int r = 0; r < MPC_CS_NM; ++r) t[r] = zs[r] - ys[r];
        for (int i = 0; i < MPC_CS_NU; ++i) {{
            float ct = 0.0f;
            for (int r = 0; r < MPC_CS_NM; ++r) ct += C[r*MPC_CS_NU + i] * t[r];
            rhs[i] = sg*u[i] - f[i] + ru*(zu[i] - yu[i]) + rss*ct;
        }}
        /* u = K^-1 rhs */
        for (int i = 0; i < MPC_CS_NU; ++i) {{
            float acc = 0.0f;
            for (int k = 0; k < MPC_CS_NU; ++k) acc += Kinv[i*MPC_CS_NU + k] * rhs[k];
            u[i] = acc;
        }}
        /* 控制量：过松弛 + 硬箱投影 */
        for (int i = 0; i < MPC_CS_NU; ++i) {{
            const float vu = al*u[i] + al1*zu[i] + yu[i];
            zu[i] = mpc_clip(vu, lo_u[i], hi_u[i]);
            yu[i] = vu - zu[i];
        }}
        /* 状态检查点：过松弛 + 软约束 prox（越界部分按 ks 收缩） */
        for (int r = 0; r < MPC_CS_NM; ++r) {{
            float cu = 0.0f;
            for (int i = 0; i < MPC_CS_NU; ++i) cu += C[r*MPC_CS_NU + i] * u[i];
            const float vs = al*cu + al1*zs[r] + ys[r];
            const float vc = mpc_clip(vs, ls[r], us[r]);
            zs[r] = vc + (vs - vc) * ks[r];
            ys[r] = vs - zs[r];
        }}
    }}
    /* 状态被打坏（NaN/Inf）时清空热启动，这一拍输出 0（投影进箱内），
     * 下一拍从冷启动重来。正常输入下不会走到这里。 */
    if (!isfinite(zu[0])) {{
        for (int i = 0; i < MPC_CS_NU; ++i) u[i] = zu[i] = yu[i] = 0.0f;
        for (int r = 0; r < MPC_CS_NM; ++r) zs[r] = ys[r] = 0.0f;
        return mpc_clip(0.0f, lo_u[0], hi_u[0]);
    }}
    return zu[0];
}}

float mpc_cs_step(int j, const float anchor[2], float w_hat, float tau_ff0,
                  float tau_ff_end, float rho_j, const float qr_e[MPC_CS_NB],
                  const float dqr_e[MPC_CS_NB])
{{
    if (j < 0 || j >= MPC_NJ || anchor == 0 || qr_e == 0 || dqr_e == 0) return 0.0f;
    const float w = MPC_USE_WCOMP[j] ? w_hat : 0.0f;
    float ac[2];
    mpc_clamp_anchor(j, anchor, ac);
    float lo_u[MPC_CS_NU], hi_u[MPC_CS_NU], lo_s[MPC_CS_NM], hi_s[MPC_CS_NM];
    mpc_cs_control_bounds(j, tau_ff0, tau_ff_end, rho_j, lo_u, hi_u);
    mpc_cs_state_bounds(j, qr_e, dqr_e, lo_s, hi_s);
    return mpc_cs_solve(j, ac, anchor, w, lo_u, hi_u, lo_s, hi_s);
}}
"""

with open(OUT_DIR / "mpc_6axis.h", "w", encoding="utf-8", newline="\n") as f:
    f.write(header)
with open(OUT_DIR / "mpc_6axis.c", "w", encoding="utf-8", newline="\n") as f:
    f.write(source)
print("[4/4] mpc_6axis.c/h 生成")

print(f"\n[DONE] 全部生成到 {OUT_DIR}/")
print("  mpc_6axis.c/h        六轴 MPC 包装接口")
print("  rnea_6axis.c/h       RNEA 前馈")
print("  crba_6axis.c/h       惯量矩阵（P3）")
print("  sources.txt          编译清单（不再依赖 OSQP）")
