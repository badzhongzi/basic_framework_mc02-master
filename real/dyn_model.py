"""sim.py 和 gen_code.py 共用的残差模型惯量计算。

为什么不能用 M_jj（惯量矩阵的对角元）
--------------------------------------
每个关节的 MPC 只知道自己的 Δτ_j，其余关节的修正量在它眼里是扰动。
完整 RNEA 前馈下，实际闭环是

    ddq = ddq_ref + M(q)^-1 * Δτ

所以单独施加 Δτ_j 时 ddq_j = (M^-1)_jj * Δτ_j，等效惯量是

    M_eff_j = 1 / (M^-1)_jj

而不是 M_jj。由 SPD 性质总有 M_jj * (M^-1)_jj >= 1，耦合强时可以差很多倍：
openarm.urdf 在 q=0 处这个倍数是 [2.0, 19.5, 25.7, 2.5, 3.2, 1.5]。
用 M_jj 会把反馈增益低估同样的倍数，导致实际闭环主极点远大于 1
（实测关节 3 在 q=0 处 rho=5.97），表现为 Δτ 在 ±DTAU_MAX 之间 bang-bang 抖动、
位置误差不收敛。

为什么取工作空间最小值
----------------------
Ad/Bd 在嵌入式侧是编译期常数，必须一个值走遍整个工作空间。
低估 M_eff -> 过增益 -> 发散；高估 M_eff -> 欠增益 -> 变慢但仍稳定。
所以取最小值是唯一安全的方向。

M^-1 用 aba 逐列构造，不用 np.linalg.inv。
"""
import numpy as np
import pinocchio as pin


def minv_diag(model, data, q):
    """返回 (M(q)^-1) 的对角元。

    aba(q, 0, tau) = M^-1 * (tau - nle(q, 0))，减掉 tau=0 的基线即得 M^-1 * tau。
    """
    n = model.nv
    zero = np.zeros(n)
    base = np.array(pin.aba(model, data, q, zero, zero))
    out = np.empty(n)
    for j in range(n):
        tau = np.zeros(n)
        tau[j] = 1.0
        out[j] = (np.array(pin.aba(model, data, q, zero, tau)) - base)[j]
    return out


def effective_inertia(model, data, q):
    """M_eff_j = 1/(M(q)^-1)_jj，即单独驱动关节 j 时看到的等效惯量。"""
    md = minv_diag(model, data, q)
    if not np.all(np.isfinite(md)) or np.any(md <= 0.0):
        raise ValueError(f"inverse inertia diagonal must be finite and positive, got {md}")
    return 1.0 / md


def sample_poses(model, n_samples, seed, extra=()):
    """工作空间采样：q=0 + 指定关注点 + 均匀随机 + 强耦合关节的角点。"""
    n = model.nv
    lo = np.array(model.lowerPositionLimit, dtype=float)
    hi = np.array(model.upperPositionLimit, dtype=float)
    # 限位缺失或退化时兜底到 +-pi
    bad = ~np.isfinite(lo) | ~np.isfinite(hi) | (hi - lo < 1e-6)
    lo = np.where(bad, -np.pi, lo)
    hi = np.where(bad, np.pi, hi)

    rng = np.random.default_rng(seed)
    poses = [np.zeros(n)]
    poses.extend(np.asarray(q, dtype=float) for q in extra)
    poses.extend(lo + (hi - lo) * rng.random(n) for _ in range(n_samples))
    # 惯量耦合最强的关节额外取角点，避免随机采样漏掉最坏构型
    corner_joints = tuple(range(min(3, n)))
    for mask in range(2 ** len(corner_joints)):
        for _ in range(40):
            q = lo + (hi - lo) * rng.random(n)
            for bit, j in enumerate(corner_joints):
                q[j] = lo[j] if (mask >> bit) & 1 else hi[j]
            poses.append(q)
    return poses, lo, hi


def design_inertia(model, data, mode, n_samples=4000, seed=0, extra_poses=()):
    """返回残差模型该用的惯量，以及一份诊断信息。

    mode:
      "effective_min" —— 工作空间内 M_eff 的最小值（推荐，鲁棒稳定）
      "effective_q0"  —— 只在 q=0 处取 M_eff（不保证全工作空间稳定）
      "diagonal_q0"   —— M_jj，即旧行为（关节 2/3 会发散，仅用于对照）
    """
    n = model.nv
    q0 = np.zeros(n)
    Mu = np.array(pin.crba(model, data, q0))
    M_diag_q0 = np.diag(np.triu(Mu) + np.triu(Mu, 1).T).copy()
    M_eff_q0 = effective_inertia(model, data, q0)

    info = {
        "mode": mode,
        "M_diag_q0": M_diag_q0,
        "M_eff_q0": M_eff_q0,
        "underestimate_q0": M_diag_q0 / M_eff_q0,
    }

    if mode == "diagonal_q0":
        info["M_design"] = M_diag_q0
        return M_diag_q0.copy(), info
    if mode == "effective_q0":
        info["M_design"] = M_eff_q0
        return M_eff_q0.copy(), info
    if mode != "effective_min":
        raise ValueError(f"unknown inertia mode {mode!r}")

    poses, lo, hi = sample_poses(model, n_samples, seed, extra=extra_poses)
    meff = np.empty((len(poses), n))
    for s, q in enumerate(poses):
        meff[s] = effective_inertia(model, data, q)

    M_design = meff.min(axis=0)
    info.update({
        "n_poses": len(poses),
        "sample_lo": lo,
        "sample_hi": hi,
        "M_eff_min": M_design.copy(),
        "M_eff_max": meff.max(axis=0),
        "M_eff_median": np.median(meff, axis=0),
        "M_design": M_design.copy(),
    })
    return M_design, info


def lqr_gain(M_model, N, Q_pos, Q_vel, R, dt):
    """有限时域 LQR 逆推，等于无约束时 QP 的第一个控制量。

    返回 (K_pos, K_vel)，满足 dtau = K_pos*e_q + K_vel*e_dq。
    用来反推误差钳位的上限和线性工作范围，不参与在线求解。
    残差模型：e'' = -dtau/M，离散化 b = -dt/M。
    """
    b = -dt / M_model
    s0 = s1 = s2 = 0.0
    k0 = k1 = 0.0
    for _ in range(N):
        t0, t1, t2 = Q_pos + s0, s1, Q_vel + s2
        btb = R + b * b * t2
        k0 = b * t1 / btb
        k1 = b * (t1 * dt + t2) / btb
        p0, p1 = b * t1, b * (t1 * dt + t2)
        a00, a01 = t0, t0 * dt + t1
        a11 = t0 * dt * dt + 2.0 * t1 * dt + t2
        s0 = a00 - p0 * k0
        s1 = a01 - p0 * k1
        s2 = a11 - p1 * k1
    return -k0, -k1


def clamp_limits(M_design, N, Q_pos, Q_vel, R, dt, dtau_max, factor):
    """锚点误差钳位上限。返回 (e_max, de_max, K_pos, K_vel)，factor=None 时前两项为 None。

    E_MAX_j  = factor*dtau_max/|K_pos_j|
    DE_MAX_j = factor*dtau_max/|K_vel_j|
    即"该项单独就能让输出饱和 factor 倍"的误差。超过这个量级后 QP 的答案必然是
    顶满 DTAU_MAX，却要付出全部控制约束同时激活的收敛代价。

    仿真和代码生成必须调用同一个函数，否则上板行为和仿真不一致。
    """
    M_design = np.asarray(M_design, dtype=float)
    n = M_design.size
    K_pos = np.empty(n)
    K_vel = np.empty(n)
    for j in range(n):
        K_pos[j], K_vel[j] = lqr_gain(M_design[j], N, Q_pos, Q_vel, R, dt)
    if factor is None:
        return None, None, K_pos, K_vel
    factor = float(factor)
    if factor <= 0.0:
        raise ValueError("钳位系数必须为正或 None")
    return (factor * dtau_max / np.abs(K_pos),
            factor * dtau_max / np.abs(K_vel), K_pos, K_vel)


def effort_array(s, model):
    """关节力矩绝对上限 [Nm]。s.EFFORT_LIMIT 为 None 时取 URDF 的 effort 字段。

    仿真和 gen_code 调同一个函数，否则生成的约束和仿真验过的不是一回事。
    """
    n = model.nv
    if getattr(s, "EFFORT_LIMIT", None) is None:
        v = np.asarray(model.effortLimit, dtype=float).copy()
    else:
        v = np.asarray(s.EFFORT_LIMIT, dtype=float)
        if v.ndim == 0:
            v = np.full(n, float(v))
    if v.size != n:
        raise ValueError(f"EFFORT_LIMIT 长度必须等于关节数 {n}，got {v.size}")
    if np.any(v <= 0.0) or not np.all(np.isfinite(v)):
        raise ValueError(
            f"关节力矩上限必须是正有限值，got {v}（URDF 的 effort 没填？）")
    return v


def control_bounds(dtau_max_j, effort_j, tau_ff_j):
    """控制量 Δτ 的上下界：同时满足修正量上限和总力矩上限。

        |Δτ|          <= dtau_max_j
        |tau_ff + Δτ| <= effort_j     ->  Δτ ∈ [-eff - tau_ff, eff - tau_ff]

    两者取交集。tau_ff 在整个预测时域内按常值处理：时域只有 N*DT = 60 ms，
    而前馈是慢变量。C 侧 mpc_build_vectors() 逐行对应同一套逻辑。

    tau_ff 本身已经超过 effort + dtau_max 时交集为空，这时把区间塌成单点
    (朝着把总力矩拉回限内的方向顶满)，而不是让 QP 变成不可行问题。
    """
    lo = max(-dtau_max_j, -effort_j - tau_ff_j)
    hi = min(dtau_max_j, effort_j - tau_ff_j)
    if lo > hi:
        v = -dtau_max_j if tau_ff_j > 0.0 else dtau_max_j
        return v, v
    return lo, hi


def control_bounds_u(dtau_max_j, effort_j, tau_ff_j, rho_j):
    """QP 决策变量 u 的边界。Δτ = ρ·u，所以把 Δτ 的边界除以 ρ（ρ > 0）。"""
    lo, hi = control_bounds(dtau_max_j, effort_j, tau_ff_j)
    return lo / rho_j, hi / rho_j


def saturate_total(tau_ff, dtau, effort):
    """装配总力矩并按 effort 饱和。返回 (tau, n_saturated)。

    为什么 QP 里的约束不够：错峰调度下 Δτ 要保持 MPC_NJ/JOINTS_PER_TICK 拍，
    而 tau_ff 每拍都在变。QP 只在轮到该关节的那一拍按当时的 tau_ff 算边界，
    之后 tau_ff 变了，tau_ff(now) + Δτ(两拍前算的) 就可能顶出 effort——
    实测把 effort 调小到约束会激活时，确实超了 0.06~0.67 Nm。
    所以必须在装配处再饱和一次。两道一起用：
      QP 约束   让 MPC 知道上限，不会积出一个发不出去的修正量（抗饱和）
      装配饱和  保证真正发给电机的值在任何时候都不超限
    """
    tau_ff = np.asarray(tau_ff, dtype=float)
    dtau = np.asarray(dtau, dtype=float)
    effort = np.asarray(effort, dtype=float)
    raw = tau_ff + dtau
    tau = np.clip(raw, -effort, effort)
    return tau, int(np.count_nonzero(tau != raw))


def friction_feedforward(s, dq):
    """摩擦前馈补偿量 [Nm]。和 C 侧 mpc_friction_ff() 逐行对应。

    用的是带噪声的测量速度，所以按 FRICTION_FF_SCALE 打折不补满，
    零速附近用 tanh 平滑，理由见 mpc_config 的注释。
    """
    sc = float(s.FRICTION_FF_SCALE)
    dq = np.asarray(dq, dtype=float)
    if sc == 0.0:
        return np.zeros_like(dq)
    fc = np.asarray(s.FRICTION_COULOMB, dtype=float)
    fv = np.asarray(s.FRICTION_VISCOUS, dtype=float)
    return sc * (fc * np.tanh(dq / float(s.FRICTION_FF_DQ_EPS)) + fv * dq)


# ================================================================
# P3: 构型相关的惯量比
#
# MPC 的残差模型用的是常数 M_design（工作空间最小值），QP 矩阵因此固定。
# 实际惯量随构型变，J0 能差 46.8 倍，按最小值设计等于在重构型下增益偏低
# 46.8 倍。做法：QP 的决策变量 u 解释成"按设计惯量换算的力矩"，
# 真正下发的修正量是 Δτ = ρ(q)·u，ρ = M_eff(q)/M_design。
# 这样闭环 ë = -(Δτ/M_eff(q) + w) = -(u/M_design + w)，对 u 来说模型和
# 设计点完全一样（线性定常），QP 的矩阵、条件数、迭代特性都不变，
# 只是在线多了一个乘法和边界的缩放。
# ================================================================

def minv_diag_chol(M):
    """对称正定 M 的逆矩阵对角元，用 Cholesky 手写（和 C 侧逐行对应）。

    不用 np.linalg：这个环境里 np.linalg 会原生崩溃。
    M = L L^T，X = L^{-1}，M^{-1} = X^T X，所以 (M^{-1})_jj = sum_i X_ij^2。
    """
    M = np.asarray(M, dtype=float)
    n = M.shape[0]
    L = np.zeros((n, n))
    for i in range(n):
        for k in range(i + 1):
            acc = M[i, k]
            for p in range(k):
                acc -= L[i, p] * L[k, p]
            if i == k:
                if not acc > 0.0:
                    raise ValueError("惯量矩阵不是正定的")
                L[i, i] = np.sqrt(acc)
            else:
                L[i, k] = acc / L[k, k]
    X = np.zeros((n, n))
    for i in range(n):
        X[i, i] = 1.0 / L[i, i]
        for k in range(i):
            acc = 0.0
            for p in range(k, i):
                acc += L[i, p] * X[p, k]
            X[i, k] = -acc / L[i, i]
    return (X * X).sum(axis=0)


def inertia_ratio(model, data, q, M_design, rho_lo, rho_hi):
    """ρ_j = M_eff_j(q) / M_design_j，M_eff = 1/(M^-1)_jj，钳到 [rho_lo, rho_hi]。

    钳位是安全网：高估惯量 = 过增益 = 可能发散，所以上限要有余量但不能放开。
    """
    Mu = np.asarray(pin.crba(model, data, np.asarray(q, dtype=float)))
    M = np.triu(Mu) + np.triu(Mu, 1).T          # crba 只填上三角
    meff = 1.0 / minv_diag_chol(M)
    return np.clip(meff / np.asarray(M_design), rho_lo, rho_hi)


def inertia_ratio_limits(s, info):
    """ρ 的上下限。下限 RATIO_MIN（M_design 是采样最小值，再小只可能是采样漏点）；
    上限 = 采样最大值 / M_design * MARGIN。"""
    M_design = np.asarray(info["M_design"], dtype=float)
    lo = np.full(M_design.size, float(s.INERTIA_RATIO_MIN))
    if "M_eff_max" in info:
        hi = np.asarray(info["M_eff_max"]) / M_design * float(s.INERTIA_RATIO_MAX_MARGIN)
    else:
        hi = np.full(M_design.size, 1.0)
    return lo, np.maximum(hi, lo)


# ================================================================
# P1 + P2: 扰动观测器 + 延迟补偿
#
# 每个关节一个 3 状态观测器 z = [e, de, w]，e = q_ref - q（误差），
# w 是"模型没解释的加速度"（负载、摩擦残差、质量偏差、关节耦合都在里面）：
#     e⁺  = e + T·de - T²/2·(u/M_d + w)
#     de⁺ = de - T·(u/M_d + w)
#     w⁺  = w                              （随机游走）
# 测量 e 和 de（都带量化噪声），用稳态 Kalman 增益融合。
#
# 延迟补偿：反馈晚 d_fb 拍到、力矩晚 d_ctrl 拍生效，总共 D = d_fb + d_ctrl。
# 观测器在"测量时刻"更新，再用已经发出去的 D 个 u 把状态前推到
# "这一拍的力矩真正生效的时刻"，MPC 的锚点用前推后的状态。
# 需要的 u 都在历史里，所以前推是精确的（模型误差除外）。
# ================================================================

def _mm(A, B):
    """小矩阵乘矩阵，不走 BLAS。

    这个环境里 numpy 的 2D @ 2D 会调 BLAS 的 dgemm，而那个 DLL 是延迟加载的、
    加载失败（Windows 异常码 0xc06d007f），进程直接退出、try/except 抓不到。
    以前记成"np.linalg.inv 偶发崩溃"，根因其实是这个。矩阵乘向量走另一条路，没事。
    """
    return (np.asarray(A)[:, :, None] * np.asarray(B)[None, :, :]).sum(axis=1)


def observer_matrices(T, m_model):
    """单关节观测器的状态转移 F (3x3) 和输入向量 G (3,)，输入是 u。"""
    F = np.array([[1.0, T, -0.5 * T * T],
                  [0.0, 1.0, -T],
                  [0.0, 0.0, 1.0]])
    G = np.array([-0.5 * T * T / m_model, -T / m_model, 0.0])
    return F, G


def kalman_gain(F, Q, R, iters=50000):
    """稳态 Kalman 增益 L (3x2)，测量矩阵 H = [I2 0]。

    迭代离散 Riccati 方程。S 是 2x2，用闭式求逆（np.linalg 会原生崩溃）。
    """
    P = np.eye(3) * 1e-6
    L = np.zeros((3, 2))
    for _ in range(iters):
        Pm = _mm(_mm(F, P), F.T) + Q
        S = Pm[:2, :2] + R
        det = S[0, 0] * S[1, 1] - S[0, 1] * S[1, 0]
        Si = np.array([[S[1, 1], -S[0, 1]], [-S[1, 0], S[0, 0]]]) / det
        Ln = _mm(Pm[:, :2], Si)
        Pn = Pm - _mm(Ln, Pm[:2, :])
        done = np.max(np.abs(Ln - L)) < 1e-13 * max(1.0, np.max(np.abs(Ln)))
        P, L = Pn, Ln
        if done:
            break
    return L


def dist_comp_mask(s, n):
    """USE_DIST_COMP 可以是一个 bool（全部关节），也可以是逐关节的列表。"""
    v = s.USE_DIST_COMP
    if isinstance(v, (bool, np.bool_)):
        return np.full(n, bool(v))
    v = np.asarray(v, dtype=bool)
    if v.size != n:
        raise ValueError(f"USE_DIST_COMP 列表长度必须等于关节数 {n}，got {v.size}")
    return v


def observer_design(s, M_model):
    """返回逐关节的 (F, G, L)，形状 (n,3,3) (n,3) (n,3,2)。

    噪声参数用力矩单位给（更直观），按 M_model 换算成加速度：
      OBS_ACC_NOISE_NM  模型短期误差（白噪声加速度），越大越信测量
      OBS_DIST_RW_NM    扰动每拍的随机游走幅度，越大 ŵ 跟得越快、噪声越大
    测量噪声直接用编码器分辨率（NOISE_*_LSB）：量化 + 抖动的方差约 LSB²/6。
    """
    T = float(s.BASE_DT)
    M_model = np.asarray(M_model, dtype=float)
    n = M_model.size
    acc = np.asarray(s.OBS_ACC_NOISE_NM, dtype=float)
    rw = np.asarray(s.OBS_DIST_RW_NM, dtype=float)
    lq = np.asarray(s.NOISE_Q_LSB, dtype=float)
    ldq = np.asarray(s.NOISE_DQ_LSB, dtype=float)
    Fs = np.zeros((n, 3, 3))
    Gs = np.zeros((n, 3))
    Ls = np.zeros((n, 3, 2))
    for j in range(n):
        F, G = observer_matrices(T, M_model[j])
        sa = acc[j] / M_model[j]
        sr = rw[j] / M_model[j]
        Q = sa * sa * np.array([[T ** 4 / 4, T ** 3 / 2, 0.0],
                                [T ** 3 / 2, T ** 2, 0.0],
                                [0.0, 0.0, 0.0]])
        Q[2, 2] = sr * sr
        Q = Q + np.eye(3) * 1e-18
        R = np.diag([lq[j] ** 2 / 6.0, ldq[j] ** 2 / 6.0])
        Fs[j], Gs[j], Ls[j] = F, G, kalman_gain(F, Q, R)
    return Fs, Gs, Ls


class Estimator:
    """逐关节的观测器 + 延迟补偿。和 C 侧 mpc_est_* 逐行对应。

    hist[j] 是长度 D+1 的移位寄存器：tick k 调用 update/anchor 时
    hist[j][0] = u[k-1-D]（作用在"测量时刻前一拍"的那个），
    hist[j][1..D] = u[k-D .. k-1]（前推用的）。commit 在本拍末尾把 u[k] 推进去。
    """

    def __init__(self, F, G, L, D):
        self.F, self.G, self.L = F, G, L
        self.D = int(D)
        n = F.shape[0]
        self.z = np.zeros((n, 3))
        self.hist = np.zeros((n, self.D + 1))

    def update(self, j, e_meas, de_meas):
        zp = self.F[j] @ self.z[j] + self.G[j] * self.hist[j, 0]
        inn = np.array([e_meas - zp[0], de_meas - zp[1]])
        self.z[j] = zp + self.L[j] @ inn

    def anchor(self, j):
        """前推到力矩生效时刻，返回 ([e, de], ŵ)。"""
        z = self.z[j].copy()
        for i in range(1, self.D + 1):
            z = self.F[j] @ z + self.G[j] * self.hist[j, i]
        return z[:2].copy(), float(z[2])

    def commit(self, j, u_applied):
        self.hist[j, :-1] = self.hist[j, 1:]
        self.hist[j, -1] = u_applied


def closed_loop_pole(M_plant, K_pos, K_vel, h):
    """反馈每 h 秒更新并保持（精确 ZOH）下的闭环主极点与 2% 沉降时间。"""
    c00 = 1.0 - 0.5 * h * h / M_plant * K_pos
    c01 = h - 0.5 * h * h / M_plant * K_vel
    c10 = -h / M_plant * K_pos
    c11 = 1.0 - h / M_plant * K_vel
    tr, det = c00 + c11, c00 * c11 - c01 * c10
    disc = tr * tr - 4.0 * det
    if disc >= 0.0:
        r = np.sqrt(disc)
        ev = np.array([(tr + r) / 2.0, (tr - r) / 2.0], dtype=complex)
    else:
        r = np.sqrt(-disc)
        ev = np.array([complex(tr / 2.0, r / 2.0), complex(tr / 2.0, -r / 2.0)])
    rho = float(np.abs(ev).max())
    ts = float("inf") if rho >= 1.0 else h * np.log(0.02) / np.log(rho)
    return rho, ts


def print_inertia_report(info):
    """打印惯量选型诊断。sim.py 和 gen_code.py 用同一份输出便于比对。"""
    np.set_printoptions(precision=5, suppress=True, linewidth=150)
    print(f"     惯量模型 mode = {info['mode']}")
    print(f"     M_jj(q=0)     = {info['M_diag_q0']}")
    print(f"     M_eff(q=0)    = {info['M_eff_q0']}")
    print(f"     低估倍数(q=0) = {info['underestimate_q0']}   (1=解耦良好)")
    if "M_eff_min" in info:
        print(f"     采样 {info['n_poses']} 个构型:")
        print(f"     M_eff_min     = {info['M_eff_min']}")
        print(f"     M_eff_median  = {info['M_eff_median']}")
        print(f"     M_eff_max     = {info['M_eff_max']}")
    print(f"     残差模型采用  = {info['M_design']}")
