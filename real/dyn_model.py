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
