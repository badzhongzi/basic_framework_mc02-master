# gen_code.py
# 生成 STM32 可用的 C 代码
import warnings
warnings.filterwarnings("ignore")

import os
import shutil
from pathlib import Path
import numpy as np
import scipy.sparse as sp
import casadi as ca
import pinocchio as pin
import pinocchio.casadi as cpin
import osqp
import mpc_config as cfg
import dyn_model as dyn

# ================== 配置 ==================
URDF_PATH   = cfg.URDF_PATH
SCRIPT_DIR  = Path(__file__).resolve().parent
OUT_DIR     = SCRIPT_DIR / "generated"

BASE_DT     = cfg.BASE_DT
N           = cfg.N
CASADI_REAL = "float"

Q_pos    = cfg.Q_POS
Q_vel    = cfg.Q_VEL
R_weight = cfg.R_WEIGHT
dtau_max = cfg.DTAU_MAX

RHO      = cfg.OSQP_RHO
EPS_ABS  = cfg.OSQP_EPS_ABS
EPS_REL  = cfg.OSQP_EPS_REL
MAX_ITER = cfg.OSQP_MAX_ITER
CHECK_TERMINATION = cfg.OSQP_CHECK_TERMINATION
# ==========================================

os.makedirs(OUT_DIR, exist_ok=True)

# ---------- 1. 加载 URDF ----------
model  = pin.buildModelFromUrdf(URDF_PATH)
data   = model.createData()
cmodel = cpin.Model(model)
cdata  = cmodel.createData()
n = model.nv
assert model.nq == n, "只支持固定基、全旋转关节"
if n != 6:
    raise ValueError(f"current generator requires a 6-axis URDF, got {n} velocity DOFs")
print(f"[1/4] URDF OK, n = {n}")

if not 1 <= cfg.JOINTS_PER_TICK <= n:
    raise ValueError("JOINTS_PER_TICK 必须在 1 和关节数之间")
if n % cfg.JOINTS_PER_TICK != 0:
    raise ValueError("为保证固定错峰周期，关节数必须能被 JOINTS_PER_TICK 整除")
scheduled_mpc_dt = BASE_DT * n / cfg.JOINTS_PER_TICK
if cfg.MPC_DT_OVERRIDE is None:
    DT = scheduled_mpc_dt
else:
    DT = cfg.MPC_DT_OVERRIDE
    if DT <= 0.0 or not np.isclose(DT, scheduled_mpc_dt, rtol=1e-6, atol=1e-12):
        raise ValueError(
            "MPC_DT_OVERRIDE must equal the staggered per-joint update period "
            f"{scheduled_mpc_dt:.9g} s; adjust JOINTS_PER_TICK instead"
        )

# ---------- 2. 在 q=0 处算常数 a10_j, b1_j ----------
q0 = np.zeros(n)
G0 = pin.rnea(model, data, q0, np.zeros(n), np.zeros(n))
eps = 1e-6
dG_dq = np.zeros(n)
for i in range(n):
    qp = q0.copy(); qp[i] += eps
    Gp = pin.rnea(model, data, qp, np.zeros(n), np.zeros(n))
    dG_dq[i] = (Gp[i] - G0[i]) / eps

# 残差模型的惯量：用等效惯量 1/(M^-1)_jj，不是 M_jj。理由见 dyn_model.py。
# 必须和 sim.py 用同一个 INERTIA_MODE/SAMPLES/SEED，否则仿真和上板行为不一致。
M_design, inertia_info = dyn.design_inertia(
    model, data, cfg.INERTIA_MODE,
    n_samples=cfg.INERTIA_SAMPLES, seed=cfg.INERTIA_SEED,
    extra_poses=(np.array(cfg.Q_GOAL, dtype=float),))
dyn.print_inertia_report(inertia_info)

if not np.all(np.isfinite(M_design)) or np.any(M_design <= 0.0):
    raise ValueError(f"design inertia must be finite and positive, got {M_design}")
# 完整 RNEA 前馈已经补偿名义重力时，不在残差模型中重复加入重力梯度。
a10  = (-dG_dq / M_design * DT if cfg.USE_GRAVITY_GRADIENT_MODEL
        else np.zeros(n))       # Ad[1,0]
b1   = -DT / M_design           # Bd[1]   = -dt/M_eff
print(f"     a10  = {a10}")
print(f"     b1   = {b1}")

# 锚点误差钳位上限。和仿真调用同一个 dyn.clamp_limits，保证上板行为一致。
# 不钳位时关节 6 最坏要 1000 次 ADMM 迭代，1 ms 预算装不下。
E_MAX, DE_MAX, K_pos, K_vel = dyn.clamp_limits(
    M_design, N, Q_pos, Q_vel, R_weight, DT, dtau_max, cfg.E_CLAMP_FACTOR)
CLAMP_ON = E_MAX is not None
print(f"     K_pos  = {K_pos}")
print(f"     K_vel  = {K_vel}")
if CLAMP_ON:
    if not (np.all(np.isfinite(E_MAX)) and np.all(E_MAX > 0.0)
            and np.all(np.isfinite(DE_MAX)) and np.all(DE_MAX > 0.0)):
        raise ValueError(f"clamp limits must be finite and positive, got {E_MAX}, {DE_MAX}")
    print(f"     E_MAX  = {E_MAX}   (factor={cfg.E_CLAMP_FACTOR:g})")
    print(f"     DE_MAX = {DE_MAX}")
else:
    print("     误差钳位关闭 (E_CLAMP_FACTOR=None)，迭代数无上界保障")

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

# ---------- 4. 为每个关节构造固定 QP 并生成独立 workspace ----------
nz  = 3 * N                # [e_1..e_N, Δτ_0..Δτ_{N-1}]
n_c = 3 * N                # 2N 动力学等式 + N 控制约束

# OSQP 目标为 0.5*z.T*P*z + q.T*z。这里与仿真保持一致，
# 使其对应 Σ(e-delta).T*Q*(e-delta) + Σ(Δτ).T*R*(Δτ)。
P_diag = np.zeros(nz)
for k in range(N):
    P_diag[k*2]     = cfg.QP_COST_FACTOR * Q_pos
    P_diag[k*2 + 1] = cfg.QP_COST_FACTOR * Q_vel
for k in range(N):
    P_diag[2*N + k] = cfg.QP_COST_FACTOR * R_weight
P = sp.diags(P_diag).tocsc()

def build_qp_matrix(Ad, Bd):
    """构造 e_(k+1)-Ad*e_k-Bd*u_k=0 的稀疏约束矩阵。"""
    rows, cols, vals = [], [], []
    for k in range(N):
        for i in range(2):
            row = k*2 + i
            rows.append(row); cols.append(k*2 + i); vals.append(1.0)
            if k > 0:
                for jj in range(2):
                    rows.append(row); cols.append((k-1)*2 + jj)
                    vals.append(-Ad[i, jj])
            rows.append(row); cols.append(2*N + k); vals.append(-Bd[i, 0])
    for k in range(N):
        row = 2*N + k
        rows.append(row); cols.append(2*N + k); vals.append(1.0)
    return sp.csc_matrix((vals, (rows, cols)), shape=(n_c, nz)).tocsc()

l = np.zeros(n_c); u = np.zeros(n_c)
for k in range(N):
    l[2*N + k] = -dtau_max
    u[2*N + k] =  dtau_max

solver_dir = OUT_DIR / "generated_solver"
if solver_dir.exists():
    shutil.rmtree(solver_dir)
os.makedirs(solver_dir, exist_ok=True)
for j in range(n):
    Ad_j = np.array([[1.0, DT], [a10[j], 1.0]])
    Bd_j = np.array([[0.0], [b1[j]]])
    A_j = build_qp_matrix(Ad_j, Bd_j)
    prob = osqp.OSQP()
    prob.setup(P, np.zeros(nz), A_j, l, u,
               rho=RHO, eps_abs=EPS_ABS, eps_rel=EPS_REL,
               max_iter=MAX_ITER,
               check_termination=CHECK_TERMINATION, verbose=False,
               warm_start=cfg.OSQP_WARM_START,
               adaptive_rho=False,
               polish=False)
    joint_dir = solver_dir / f"joint{j}"
    os.makedirs(joint_dir, exist_ok=True)
    prob.codegen(joint_dir, prefix=f"mpc_j{j}", parameters="matrices",
                 force_rewrite=True, use_float=True)
print("[3/4] 六个独立 OSQP solver workspace 生成")

# ---------- 5. 生成统一的 mpc_6axis.c/h 包装接口 ----------
# Each OSQP codegen directory contains a copy of the common solver sources.
# Compile the common implementation exactly once to avoid duplicate symbols.
common_dir = solver_dir / "common"
if n > 0:
    shutil.copytree(solver_dir / "joint0" / "inc", common_dir / "inc", dirs_exist_ok=True)
    shutil.copytree(solver_dir / "joint0" / "src", common_dir / "src", dirs_exist_ok=True)
    shutil.copy2(solver_dir / "joint0" / "osqp_configure.h",
                 common_dir / "osqp_configure.h")

source_manifest = [
    "# Run these paths from the generated/ directory:",
    "# Compile the common OSQP sources exactly once:",
    "generated_solver/common/src/*.c",
    "# Compile one workspace source for each joint:",
]
source_manifest.extend(
    f"generated_solver/joint{j}/mpc_j{j}workspace.c" for j in range(n)
)
source_manifest.extend([
    "# Include directories:",
    "-I .",
    "-I generated_solver/common",
    "-I generated_solver/common/inc/public",
    "-I generated_solver/common/inc/private",
    "# Also compile generated/mpc_6axis.c and generated/rnea_6axis.c",
])
with open(solver_dir / "sources.txt", "w", encoding="utf-8") as manifest:
    manifest.write("\n".join(source_manifest) + "\n")

a10_str = ", ".join(f"{v:.8e}f" for v in a10)
b1_str  = ", ".join(f"{v:.8e}f" for v in b1)

# 钳位关闭时上限填 +inf，C 侧的 clamp 代码就退化成恒等变换，
# 两条分支共用一份实现，避免仿真和 C 出现结构性差异。
_e_max_vals  = E_MAX  if CLAMP_ON else np.full(n, np.inf)
_de_max_vals = DE_MAX if CLAMP_ON else np.full(n, np.inf)
_fmt = lambda v: "INFINITY" if not np.isfinite(v) else f"{v:.8e}f"
e_max_str  = ", ".join(_fmt(v) for v in _e_max_vals)
de_max_str = ", ".join(_fmt(v) for v in _de_max_vals)
clamp_note = (f"factor={cfg.E_CLAMP_FACTOR:g}" if CLAMP_ON else "关闭 (INFINITY)")
workspace_headers = "\n".join(
    f'#include "generated_solver/joint{j}/mpc_j{j}workspace.h"' for j in range(n)
)
solver_decls = "\n".join(
    f'    &mpc_j{j}solver' + (',' if j < n - 1 else '')
    for j in range(n)
)

header = f"""#ifndef MPC_6AXIS_H
#define MPC_6AXIS_H
#ifdef __cplusplus
extern "C" {{
#endif

#define MPC_NJ  {n}
#define MPC_N   {N}
#define MPC_NZ  {nz}
#define MPC_NC  {n_c}
#define MPC_BASE_DT  {BASE_DT}f
#define MPC_DT  {DT}f
#define MPC_JOINTS_PER_TICK  {cfg.JOINTS_PER_TICK}
#define MPC_DTAU_MAX  {dtau_max:.8e}f
#define MPC_MAX_ITER  {MAX_ITER}

/* 锚点误差：e_anchor,k = x_ref(t0) - x(t0+k*DT)。
 * 参考偏移 delta_k = x_ref(t0) - x_ref(t0+k*DT)，
 * 因此 e_anchor,k - delta_k 是未来时刻的普通跟踪误差。
 * 每个关节使用独立、固定 Ad/Bd 的 OSQP workspace。 */
extern const float MPC_A10[MPC_NJ];
extern const float MPC_B1[MPC_NJ];

/* 锚点误差钳位上限（{clamp_note}）。
 * 误差远超线性范围时 QP 的答案必然是顶满 DTAU_MAX，但这种输入会让全部 MPC_N 个
 * 控制约束同时激活，正是 ADMM 最慢的工况——不钳位时最坏需要 1000 次迭代，
 * 装不进 1 ms 的 tick。钳位后最坏 55 次。
 * mpc_build_vectors() 内部已自动钳位，调用方无需自己处理。 */
extern const float MPC_E_MAX[MPC_NJ];
extern const float MPC_DE_MAX[MPC_NJ];

void mpc_get_AB(int j, float *A, float *B);
void mpc_matvec2(const float *A, const float *v, float *out);
/* 把锚点误差钳到 [-MPC_E_MAX, MPC_E_MAX] x [-MPC_DE_MAX, MPC_DE_MAX]。
 * out 可以和 in 指向同一块内存。 */
void mpc_clamp_anchor(int j, const float in[2], float out[2]);

/* 构造一个关节的在线 QP 向量：
 * anchor_error = [q_ref(t0)-q(t0), dq_ref(t0)-dq(t0)]；
 * delta_seq[k] = x_ref(t0)-x_ref(t0+k*DT)，共 MPC_N+1 个点。
 * q_out、l_out、u_out 的长度分别为 MPC_NZ、MPC_NC、MPC_NC。 */
void mpc_build_vectors(int j, const float anchor_error[2],
                       const float delta_seq[MPC_N + 1][2],
                       float q_out[MPC_NZ], float l_out[MPC_NC],
                       float u_out[MPC_NC]);

/* 更新指定关节的线性项 q 和上下界 l/u。 */
int mpc_update_joint(int j, const float *q, const float *l, const float *u);
/* 求解指定关节的 QP；返回 OSQP API 错误码（0 表示调用成功）。 */
int mpc_solve_joint(int j);
/* 读取指定关节最近一次求解的 OSQP 状态码（1=solved，2=solved inaccurate）。 */
int mpc_get_status(int j);
/* 读取指定关节解中的第一个 Δτ，已钳到 ±MPC_DTAU_MAX。失败或索引非法时返回 0。
 *
 * 必须钳位：ADMM 的中间迭代点不满足约束，max_iter 用尽时取回的值可能越界。
 * 调用方的处置策略要和仿真一致（USE_UNCONVERGED_ITERATE={cfg.USE_UNCONVERGED_ITERATE}）：
 *   mpc_get_status(j) 返回 1(solved) 或 2(solved inaccurate) 时直接采用；
 *   其余情况仿真里{'也采用当前迭代点（大扰动下 hold 过期值更糟）' if cfg.USE_UNCONVERGED_ITERATE else '沿用上一周期的 Δτ'}。 */
float mpc_get_first_dtau(int j);

#ifdef __cplusplus
}}
#endif
#endif
"""

source = f"""#include <math.h>

#include "mpc_6axis.h"
#include "osqp_api_functions.h"

{workspace_headers}

/* 每个关节的离散模型系数（q=0 处线性化）。 */
const float MPC_A10[MPC_NJ] = {{ {a10_str} }};
const float MPC_B1[MPC_NJ] = {{ {b1_str} }};

/* 锚点误差钳位上限（{clamp_note}）。由 LQR 增益反推：
 * K_pos = {np.array2string(K_pos, precision=2, separator=', ')}
 * K_vel = {np.array2string(K_vel, precision=2, separator=', ')} */
const float MPC_E_MAX[MPC_NJ] = {{ {e_max_str} }};
const float MPC_DE_MAX[MPC_NJ] = {{ {de_max_str} }};

static float mpc_clamp1(float v, float lim)
{{
    if (v >  lim) return  lim;
    if (v < -lim) return -lim;
    return v;
}}

void mpc_clamp_anchor(int j, const float in[2], float out[2])
{{
    if (j < 0 || j >= MPC_NJ || in == 0 || out == 0) return;
    out[0] = mpc_clamp1(in[0], MPC_E_MAX[j]);
    out[1] = mpc_clamp1(in[1], MPC_DE_MAX[j]);
}}

static OSQPSolver *const MPC_SOLVERS[MPC_NJ] = {{
{solver_decls}
}};

void mpc_get_AB(int j, float *A, float *B)
{{
    if (j < 0 || j >= MPC_NJ || A == 0) return;
    A[0] = 1.0f; A[1] = MPC_DT;
    A[2] = MPC_A10[j]; A[3] = 1.0f;
    if (B != 0) {{
        B[0] = 0.0f; B[1] = MPC_B1[j];
    }}
}}

void mpc_matvec2(const float *A, const float *v, float *out)
{{
    if (A == 0 || v == 0 || out == 0) return;
    out[0] = A[0]*v[0] + A[1]*v[1];
    out[1] = A[2]*v[0] + A[3]*v[1];
}}

void mpc_build_vectors(int j, const float anchor_error[2],
                       const float delta_seq[MPC_N + 1][2],
                       float q_out[MPC_NZ], float l_out[MPC_NC],
                       float u_out[MPC_NC])
{{
    if (j < 0 || j >= MPC_NJ || anchor_error == 0 || delta_seq == 0 ||
        q_out == 0 || l_out == 0 || u_out == 0) return;
    float A[4];
    mpc_get_AB(j, A, 0);
    /* 钳位放在这里而不是交给调用方，这样 C 侧不可能漏掉、和仿真必然一致。 */
    float ae[2];
    mpc_clamp_anchor(j, anchor_error, ae);
    for (int k = 0; k < MPC_N; ++k) {{
        q_out[2*k]     = -{cfg.QP_COST_FACTOR:.8e}f * {Q_pos:.8e}f * delta_seq[k + 1][0];
        q_out[2*k + 1] = -{cfg.QP_COST_FACTOR:.8e}f * {Q_vel:.8e}f * delta_seq[k + 1][1];
    }}
    /* 力矩修正量的线性代价项为零，必须显式写入整个控制段。 */
    for (int k = 0; k < MPC_N; ++k) {{
        q_out[2*MPC_N + k] = 0.0f;
    }}
    for (int k = 0; k < MPC_N; ++k) {{
        const float ax0 = A[0]*delta_seq[k][0] + A[1]*delta_seq[k][1];
        const float ax1 = A[2]*delta_seq[k][0] + A[3]*delta_seq[k][1];
        float rhs0 = delta_seq[k + 1][0] - ax0;
        float rhs1 = delta_seq[k + 1][1] - ax1;
        if (k == 0) {{
            rhs0 += A[0]*ae[0] + A[1]*ae[1];
            rhs1 += A[2]*ae[0] + A[3]*ae[1];
        }}
        l_out[2*k] = u_out[2*k] = rhs0;
        l_out[2*k + 1] = u_out[2*k + 1] = rhs1;
    }}
    for (int k = 0; k < MPC_N; ++k) {{
        l_out[2*MPC_N + k] = -{dtau_max:.8e}f;
        u_out[2*MPC_N + k] =  {dtau_max:.8e}f;
    }}
}}

int mpc_update_joint(int j, const float *q, const float *l, const float *u)
{{
    if (j < 0 || j >= MPC_NJ || MPC_SOLVERS[j] == 0 ||
        MPC_SOLVERS[j]->work == 0 || q == 0 || l == 0 || u == 0) return -1;
    return (int)osqp_update_data_vec(MPC_SOLVERS[j], q, l, u);
}}

int mpc_solve_joint(int j)
{{
    if (j < 0 || j >= MPC_NJ || MPC_SOLVERS[j] == 0) return -1;
    return (int)osqp_solve(MPC_SOLVERS[j]);
}}

int mpc_get_status(int j)
{{
    if (j < 0 || j >= MPC_NJ || MPC_SOLVERS[j] == 0 ||
        MPC_SOLVERS[j]->info == 0) return -1;
    return (int)MPC_SOLVERS[j]->info->status_val;
}}

float mpc_get_first_dtau(int j)
{{
    if (j < 0 || j >= MPC_NJ || MPC_SOLVERS[j] == 0 ||
        MPC_SOLVERS[j]->solution == 0 ||
        MPC_SOLVERS[j]->solution->x == 0) return 0.0f;
    const float v = (float)MPC_SOLVERS[j]->solution->x[2 * MPC_N];
    if (!isfinite(v)) return 0.0f;
    return mpc_clamp1(v, MPC_DTAU_MAX);
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
print("  generated_solver/joint0..joint5/  独立 OSQP 求解器")
