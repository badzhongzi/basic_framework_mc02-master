"""P7: 消元 + 定迭代次数 ADMM 的单关节 MPC 求解器（仿真和 gen_code 共用）。

为什么要换掉 OSQP
-----------------
OSQP 是通用稀疏求解器：30 个变量（20 个状态 + 10 个控制量）、20 条等式约束，
每次迭代要做稀疏 LDL 回代；迭代次数不固定（25~100），H7 上最坏要 2~3 ms
（估算，没实测），塞不下"每拍解 6 个关节"（P4）。

这里的做法
----------
1. 消元：状态 w_k 完全由 w_0 和 u 决定（w = Φ w_0 + Γ u + Ψ ŵ），代掉之后
   QP 只剩控制量 u。代价矩阵 H 是常数（P3 让模型对 u 线性定常，这一步才成立）。
2. 分块：60 ms 时域、2 ms 一拍共 30 拍，但只用 NU = 10 个控制量，
   分块长度 MPC_BLOCKS（第一块 1 拍，和每拍重解一致）。
3. 约束：
     控制量 u        硬箱约束（DTAU_MAX ∩ 沿时域预测的总力矩上限），每次迭代都投影
     位置、速度       软约束（高权重二次罚），只在每块末尾检查
   硬约束保证发出去的力矩一定不超限；软约束保证起点就越界这种不可行情况
   下求解器也不会发散（对偶变量有界）。
4. ADMM 定迭代次数：所有矩阵离线求好（包括 K^-1），在线每次迭代只有
   几次小矩阵乘向量和逐元素投影，耗时是常数。

参考预览在这里才真正起作用：代价只看跟踪误差（预览在代价里会被消掉，
已经证明过），而位置/速度/力矩约束用的是**绝对量**，要靠未来的参考来算边界。
"""
import numpy as np

import dyn_model as dyn


# ---------------------------------------------------------------- 小工具
def _inv_spd(K):
    """对称正定小矩阵求逆（Cholesky），不走 np.linalg（会原生崩溃）。"""
    n = K.shape[0]
    L = np.zeros((n, n))
    for i in range(n):
        for k in range(i + 1):
            acc = K[i, k] - float(np.dot(L[i, :k], L[k, :k]))
            if i == k:
                if not acc > 0.0:
                    raise ValueError("矩阵不是正定的")
                L[i, i] = np.sqrt(acc)
            else:
                L[i, k] = acc / L[k, k]
    X = np.zeros((n, n))                     # X = L^-1
    for i in range(n):
        X[i, i] = 1.0 / L[i, i]
        for k in range(i):
            X[i, k] = -float(np.dot(L[i, k:i], X[k:i, k])) / L[i, i]
    return dyn._mm(X.T, X)


def _mv(A, x):
    """矩阵乘向量（不走 BLAS 的 dgemm）。"""
    return (np.asarray(A) * np.asarray(x)[None, :]).sum(axis=1)


# ---------------------------------------------------------------- 离线设计
def design(s, M_design):
    """为每个关节生成求解器常数。返回 dict of arrays（第一维是关节）。

    单位：u 是"按设计惯量换算的力矩"（P3），w 是误差 [e, de]，ŵ 是扰动加速度。
    """
    T = float(s.BASE_DT)
    blocks = np.asarray(s.MPC_BLOCKS, dtype=int)
    NU = blocks.size
    NS = int(blocks.sum())                   # 预测步数
    ends = np.cumsum(blocks)                 # 每块末尾是第几步（1 起）
    NB = NU                                  # 约束检查点 = 块末尾
    M_design = np.asarray(M_design, dtype=float)
    n = M_design.size

    qp = np.broadcast_to(np.asarray(s.CS_Q_POS, dtype=float), (n,))
    qv = np.broadcast_to(np.asarray(s.CS_Q_VEL, dtype=float), (n,))
    rw = np.broadcast_to(np.asarray(s.CS_R_WEIGHT, dtype=float), (n,))
    wpos = float(s.SOFT_POS_WEIGHT)
    wvel = float(s.SOFT_VEL_WEIGHT)
    rho_u = float(s.ADMM_RHO_U)
    rho_s = float(s.ADMM_RHO_S)
    sigma = float(s.ADMM_SIGMA)

    out = {k: [] for k in ("Kinv", "C", "rs", "Gw", "Gc", "Pw", "Pc",
                           "H", "K0", "Kc", "wsoft", "scale",
                           "rho_u", "rho_s", "cond")}
    for j in range(n):
        A = np.array([[1.0, T], [0.0, 1.0]])
        B = np.array([-0.5 * T * T, -T]) / M_design[j]
        Cw = np.array([-0.5 * T * T, -T])    # ŵ 的输入向量
        # 逐步模拟：w_k = Phi_k w0 + Gam_k u + Psi_k ŵ，k = 1..NS
        Phi = np.zeros((NS, 2, 2))
        Gam = np.zeros((NS, 2, NU))
        Psi = np.zeros((NS, 2))
        Pk = np.eye(2)
        Gk = np.zeros((2, NU))
        Sk = np.zeros(2)
        blk_of_step = np.repeat(np.arange(NU), blocks)
        for k in range(NS):
            b = blk_of_step[k]
            Pk = dyn._mm(A, Pk)
            Gk = dyn._mm(A, Gk)
            Gk[:, b] += B
            Sk = _mv(A, Sk) + Cw
            Phi[k], Gam[k], Psi[k] = Pk, Gk, Sk
        # 代价 J = sum_k (qp e_k² + qv de_k²) + r * sum_i L_i u_i²
        #   = ½ uᵀ H u + fᵀ u + const，H = 2(ΓᵀQΓ + R_b)，f = Gw w0 + Gc ŵ
        H = np.zeros((NU, NU))
        Gw = np.zeros((NU, 2))
        Gc = np.zeros(NU)
        for k in range(NS):
            Qk = np.diag([qp[j], qv[j]])
            QG = dyn._mm(Qk, Gam[k])
            H += 2.0 * dyn._mm(Gam[k].T, QG)
            Gw += 2.0 * dyn._mm(QG.T, Phi[k])
            Gc += 2.0 * _mv(QG.T, Psi[k])
        H += 2.0 * np.diag(rw[j] * blocks)
        # 整体缩放：只为让 ρ 的取值和关节无关，不改变最优解
        sc = float(np.mean(np.diag(H)))
        Hs, Gws, Gcs = H / sc, Gw / sc, Gc / sc
        # 约束检查点：每块末尾的位置、速度两行
        Cr = np.zeros((2 * NB, NU))
        Pw = np.zeros((2 * NB, 2))
        Pc = np.zeros(2 * NB)
        for i, e in enumerate(ends):
            Cr[2 * i:2 * i + 2] = Gam[e - 1]
            Pw[2 * i:2 * i + 2] = Phi[e - 1]
            Pc[2 * i:2 * i + 2] = Psi[e - 1]
        rs = 1.0 / np.sqrt((Cr * Cr).sum(axis=1))      # 行归一化
        C = Cr * rs[:, None]
        # 软约束权重换到缩放后的行、缩放后的代价上
        wraw = np.tile([wpos, wvel], NB)
        wsoft = wraw / (rs * rs) / sc
        # ADMM 步长 ρ 按关节自动定：经验上取 sqrt(λmin·λmax) 附近收敛最快。
        # 各关节 H 的条件数从 10 到 6000 不等（手腕惯量小、长块之间强相关），
        # 一个全局的 ρ 对手腕慢得不可用。配置里的 ADMM_RHO_* 是乘在它上面的系数。
        lmax = _lam_max(Hs)
        lmin = 1.0 / _lam_max(_inv_spd(Hs))
        rj = np.sqrt(lmin * lmax)
        ru_j, rs_j = rho_u * rj, rho_s * rj
        K = Hs + (sigma + ru_j) * np.eye(NU) + rs_j * dyn._mm(C.T, C)
        Kinv = _inv_spd(K)
        out["rho_u"].append(ru_j)
        out["rho_s"].append(rs_j)
        out["cond"].append(lmax / lmin)
        # 不受约束时的首个控制量 u0 = K0·w0 + Kc·ŵ（给锚点钳位和诊断用）
        Hinv = _inv_spd(H)
        K0 = -dyn._mm(Hinv, Gw)[0]
        Kc = -_mv(Hinv, Gc)[0]
        for k_, v_ in (("Kinv", Kinv), ("C", C), ("rs", rs), ("Gw", Gws),
                       ("Gc", Gcs), ("Pw", Pw), ("Pc", Pc), ("H", Hs),
                       ("K0", K0), ("Kc", Kc), ("wsoft", wsoft), ("scale", sc)):
            out[k_].append(v_)
    res = {k: np.asarray(v) for k, v in out.items()}
    res.update(blocks=blocks, ends=ends, NU=NU, NB=NB, NS=NS, T=T,
               sigma=sigma, alpha=float(s.ADMM_ALPHA), iters=int(s.ADMM_ITERS))
    return res


def clamp_limits(s, d, dtau_max):
    """锚点钳位上限，用新求解器自己的无约束增益反推（含义和旧版一样）。

    只钳进代价的那个锚点，不钳进约束预测的那个：约束要按真实状态预测位置。
    """
    if s.E_CLAMP_FACTOR is None:
        return None, None
    f = float(s.E_CLAMP_FACTOR)
    return (f * dtau_max / np.abs(d["K0"][:, 0]),
            f * dtau_max / np.abs(d["K0"][:, 1]))


def joint_limits(s, model):
    """P5 状态约束用的限位（已经扣掉余量）。"""
    n = model.nv
    q_lo = np.asarray(s.JOINT_Q_MIN, dtype=float) + float(s.POS_LIMIT_MARGIN)
    q_hi = np.asarray(s.JOINT_Q_MAX, dtype=float) - float(s.POS_LIMIT_MARGIN)
    if s.JOINT_V_MAX is None:
        v = np.asarray(model.velocityLimit, dtype=float).copy()
    else:
        v = np.broadcast_to(np.asarray(s.JOINT_V_MAX, dtype=float), (n,)).copy()
    v_hi = v - float(s.VEL_LIMIT_MARGIN)
    if q_lo.size != n or q_hi.size != n or np.any(q_lo >= q_hi) or np.any(v_hi <= 0):
        raise ValueError(f"关节限位不合法: q [{q_lo}, {q_hi}], v {v_hi}")
    return q_lo, q_hi, v_hi


def _lam_max(A, it=2000):
    """对称正定矩阵的最大特征值（幂迭代，不走 np.linalg）。"""
    x = np.ones(A.shape[0]) / np.sqrt(A.shape[0])
    for _ in range(it):
        x = _mv(A, x)
        x /= np.sqrt(float((x * x).sum()))
    return float((x * _mv(A, x)).sum())


# ---------------------------------------------------------------- 在线：边界
def control_bounds_blocks(d, dtau_max_j, effort_j, tau_ff0, tau_ff_end, rho_j):
    """每块控制量 u 的上下界（硬约束）。

    预测时域里的前馈力矩按线性插值：tau_ff(k) = tau_ff0 + k/NS*(tau_ff_end - tau_ff0)，
    k = 0..NS-1。一块里 u 不变，所以这块的边界是块内每一步边界的交集；
    因为插值是线性的，交集只取决于块的首尾两步。
    交集为空（前馈本身已经超限）时退回这块第一步的边界，那一步已经处理过
    "塌成单点"的情况，永远不会给求解器一个空区间。
    """
    blocks, NS = d["blocks"], d["NS"]
    lo = np.empty(blocks.size)
    hi = np.empty(blocks.size)
    k0 = 0
    for i, L in enumerate(blocks):
        a, b = -np.inf, np.inf
        first = None
        for k in (k0, k0 + L - 1):
            tf = tau_ff0 + (k / NS) * (tau_ff_end - tau_ff0)
            l_, h_ = dyn.control_bounds(dtau_max_j, effort_j, tf)
            if first is None:
                first = (l_, h_)
            a, b = max(a, l_), min(b, h_)
        if a > b:
            a, b = first
        lo[i], hi[i] = a / rho_j, b / rho_j
        k0 += L
    return lo, hi


def state_bounds(d, q_ref_e, dq_ref_e, q_min, q_max, v_max):
    """每块末尾的误差状态 w = [e, de] 的上下界（软约束），来自绝对的关节限位。

    e = q_ref - q，所以 q ∈ [q_min, q_max]  <=>  e ∈ [q_ref - q_max, q_ref - q_min]
                 dq ∈ [-v_max, v_max]       <=> de ∈ [dq_ref - v_max, dq_ref + v_max]
    q_ref_e / dq_ref_e 是每块末尾时刻的参考，长度 NB——这里就是参考预览起作用的地方。
    """
    NB = d["NB"]
    lo = np.empty(2 * NB)
    hi = np.empty(2 * NB)
    lo[0::2] = q_ref_e - q_max
    hi[0::2] = q_ref_e - q_min
    lo[1::2] = dq_ref_e - v_max
    hi[1::2] = dq_ref_e + v_max
    return lo, hi


# ---------------------------------------------------------------- 在线：求解
class Solver:
    """六个关节的定迭代次数 ADMM，状态（u, z, y）跨拍保留做热启动。

    和 C 侧 mpc_cs_solve() 逐行对应。
    """

    def __init__(self, d):
        self.d = d
        n, NU, NB = d["Kinv"].shape[0], d["NU"], d["NB"]
        self.u = np.zeros((n, NU))
        self.zu = np.zeros((n, NU))
        self.yu = np.zeros((n, NU))
        self.zs = np.zeros((n, 2 * NB))
        self.ys = np.zeros((n, 2 * NB))
        self.last_iters = d["iters"]

    def solve(self, j, w0_cost, w0_true, w_hat, lo_u, hi_u, lo_s, hi_s, iters=None):
        """返回首个控制量 u0（一定在 [lo_u[0], hi_u[0]] 内）。

        w0_cost  进代价的锚点（可以已经钳过位，避免深度饱和时代价尺度失控）
        w0_true  进约束预测的锚点（不钳位：约束要按真实状态预测位置）
        w_hat    扰动加速度（不用扰动补偿的关节传 0）
        """
        d = self.d
        Kinv, C, rs = d["Kinv"][j], d["C"][j], d["rs"][j]
        ru, rs_, sg, al = d["rho_u"][j], d["rho_s"][j], d["sigma"], d["alpha"]
        f = _mv(d["Gw"][j], w0_cost) + d["Gc"][j] * w_hat
        off = _mv(d["Pw"][j], w0_true) + d["Pc"][j] * w_hat
        ls = rs * (lo_s - off)
        us = rs * (hi_s - off)
        ks = rs_ / (rs_ + d["wsoft"][j])           # 软约束 prox 的收缩系数
        u, zu, yu, zs, ys = (self.u[j], self.zu[j], self.yu[j],
                             self.zs[j], self.ys[j])
        n_it = d["iters"] if iters is None else int(iters)
        for _ in range(n_it):
            rhs = sg * u - f + ru * (zu - yu) + rs_ * _mv(C.T, zs - ys)
            u = _mv(Kinv, rhs)
            au = al * u + (1.0 - al) * zu
            asx = al * _mv(C, u) + (1.0 - al) * zs
            vu = au + yu
            vs = asx + ys
            zu = np.clip(vu, lo_u, hi_u)
            vc = np.clip(vs, ls, us)
            zs = vc + (vs - vc) * ks
            yu = vu - zu
            ys = vs - zs
        self.u[j], self.zu[j], self.yu[j], self.zs[j], self.ys[j] = u, zu, yu, zs, ys
        return float(zu[0])


def reference_solution(d, j, w0_cost, w0_true, w_hat, lo_u, hi_u, lo_s, hi_s):
    """同一个问题的高精度参考解（OSQP，变量 [u, p]，p 是软约束的越界量）。

    只用于校验 ADMM 定迭代次数下的精度，不进仿真回路。
        min ½uᵀHu + fᵀu + ½ Σ wsoft_r p_r²
        s.t. lo_u <= u <= hi_u,   ls <= C u - p <= us
    """
    import osqp
    import scipy.sparse as sp
    NU, NB = d["NU"], d["NB"]
    m = 2 * NB
    H, C, rs = d["H"][j], d["C"][j], d["rs"][j]
    f = _mv(d["Gw"][j], w0_cost) + d["Gc"][j] * w_hat
    off = _mv(d["Pw"][j], w0_true) + d["Pc"][j] * w_hat
    ls, us = rs * (lo_s - off), rs * (hi_s - off)
    P = np.zeros((NU + m, NU + m))
    P[:NU, :NU] = H
    P[NU:, NU:] = np.diag(d["wsoft"][j])
    q = np.concatenate([f, np.zeros(m)])
    Acon = np.zeros((NU + m, NU + m))
    Acon[:NU, :NU] = np.eye(NU)
    Acon[NU:, :NU] = C
    Acon[NU:, NU:] = -np.eye(m)
    l = np.concatenate([lo_u, ls])
    u = np.concatenate([hi_u, us])
    prob = osqp.OSQP()
    prob.setup(sp.csc_matrix(np.triu(P)), q, sp.csc_matrix(Acon), l, u,
               eps_abs=1e-10, eps_rel=1e-10, max_iter=200000, polish=False,
               verbose=False)
    r = prob.solve()
    return float(r.x[0])
