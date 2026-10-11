"""闭环仿真内核。sim.py（单次运行 + 画图）和 sweep_rt.py（参数扫描）共用。

锚点误差 MPC + 参考偏移 δ + 错峰 + 每关节独立 solver + 热启动，
与嵌入式版本一一对应。

run() 接受 overrides 覆盖 mpc_config 里的任意大写配置项，不会污染 cfg 模块，
所以一个进程里可以连续跑多组配置。模型加载和惯量扫描按 key 缓存，扫描时不重复算。
"""
import time

import numpy as np
import osqp
import pinocchio as pin
from scipy import sparse

import mpc_config as cfg
import dyn_model as dyn

CONFIG_KEYS = frozenset(k for k in dir(cfg) if k.isupper())

_MODEL_CACHE = {}
_INERTIA_CACHE = {}


class Settings:
    """mpc_config 的一份快照，overrides 里的键必须真实存在，防止拼错后静默失效。"""

    def __init__(self, overrides=None):
        for key in CONFIG_KEYS:
            setattr(self, key, getattr(cfg, key))
        for key, value in (overrides or {}).items():
            if key not in CONFIG_KEYS:
                raise KeyError(f"unknown config key {key!r}; 有效键见 mpc_config.py")
            setattr(self, key, value)

    def label(self, keys):
        return " ".join(f"{k}={getattr(self, k)}" for k in keys)


def load_model(urdf_path):
    """pinocchio 模型按路径缓存。data 每次新建，避免跨 run 共享可变状态。"""
    if urdf_path not in _MODEL_CACHE:
        _MODEL_CACHE[urdf_path] = pin.buildModelFromUrdf(urdf_path)
    model = _MODEL_CACHE[urdf_path]
    return model, model.createData()


def design_inertia(s, model, data):
    """惯量选型结果按影响它的配置项缓存。"""
    key = (s.URDF_PATH, s.INERTIA_MODE, s.INERTIA_SAMPLES, s.INERTIA_SEED,
           tuple(s.Q_GOAL))
    if key not in _INERTIA_CACHE:
        _INERTIA_CACHE[key] = dyn.design_inertia(
            model, data, s.INERTIA_MODE,
            n_samples=s.INERTIA_SAMPLES, seed=s.INERTIA_SEED,
            extra_poses=(np.array(s.Q_GOAL, dtype=float),))
    return _INERTIA_CACHE[key]


def resolve_mpc_dt(s, n_joints):
    """错峰调度决定的单关节更新周期，并校验 MPC_DT_OVERRIDE 与之一致。"""
    if not 1 <= s.JOINTS_PER_TICK <= n_joints:
        raise ValueError("JOINTS_PER_TICK 必须在 1 和关节数之间")
    if n_joints % s.JOINTS_PER_TICK != 0:
        raise ValueError("为保证固定错峰周期，关节数必须能被 JOINTS_PER_TICK 整除")
    scheduled = s.BASE_DT * n_joints / s.JOINTS_PER_TICK
    if s.MPC_DT_OVERRIDE is None:
        return scheduled
    if s.MPC_DT_OVERRIDE <= 0.0 or not np.isclose(
            s.MPC_DT_OVERRIDE, scheduled, rtol=1e-6, atol=1e-12):
        raise ValueError(
            "MPC_DT_OVERRIDE must equal the staggered per-joint update period "
            f"{scheduled:.9g} s; adjust JOINTS_PER_TICK instead")
    return s.MPC_DT_OVERRIDE


def make_reference(s, n_joints):
    """五次多项式点到点轨迹。返回 get_reference(t) -> (q_ref, dq_ref, ddq_ref)。"""
    q_init = np.array(s.Q_INIT, dtype=float)
    q_goal = np.array(s.Q_GOAL, dtype=float)
    if q_init.size != n_joints or q_goal.size != n_joints:
        raise ValueError("Q_INIT/Q_GOAL 长度必须等于关节数")
    T = float(s.T_TRAJ)
    span = q_goal - q_init
    zero = np.zeros(n_joints)

    def get_reference(t):
        if t >= T:
            return q_goal.copy(), zero.copy(), zero.copy()
        u = t / T
        h = 10 * u**3 - 15 * u**4 + 6 * u**5
        dh = (30 * u**2 - 60 * u**3 + 30 * u**4) / T
        ddh = (60 * u - 180 * u**2 + 120 * u**3) / T**2
        return q_init + span * h, span * dh, span * ddh

    return get_reference, q_init, q_goal


def build_qp_matrix(Ad, Bd, N, nz, n_con):
    """e_(k+1) - Ad*e_k - Bd*u_k = 0 的约束矩阵，外加 N 行控制盒约束。"""
    rows = []
    for k in range(N):
        for i in range(2):
            row = np.zeros(nz)
            row[k * 2 + i] = 1.0
            if k > 0:
                row[(k - 1) * 2: k * 2] = -Ad[i, :]
            row[2 * N + k] = -Bd[i, 0]
            rows.append(row)
    for k in range(N):
        row = np.zeros(nz)
        row[2 * N + k] = 1.0
        rows.append(row)
    return sparse.csc_matrix(np.vstack(rows))


def build_p_matrix(s, N, nz):
    """固定的二次代价对角。QP_COST_FACTOR=2 时对应未省略 0.5 的代价。"""
    diag = np.zeros(nz)
    for k in range(N):
        diag[k * 2] = s.QP_COST_FACTOR * s.Q_POS
        diag[k * 2 + 1] = s.QP_COST_FACTOR * s.Q_VEL
    for k in range(N):
        diag[2 * N + k] = s.QP_COST_FACTOR * s.R_WEIGHT
    return sparse.diags(diag).tocsc()


def build_vectors(s, N, nz, n_con, Ad, anchor_error, delta_seq):
    """在线 QP 向量。与生成的 C 版 mpc_build_vectors() 逐行对应。

    锚点误差 e_anchor = x_ref(t0) - x(t)，与普通跟踪误差不同。
    由 r_k = e_anchor,k - delta_k 得 e_(k+1) = A*e_k + B*u_k + delta_(k+1) - A*delta_k，
    第一步还要把当前锚点误差 A*e_0 移到等式右端。
    """
    l = np.zeros(n_con)
    u = np.zeros(n_con)
    q_vec = np.zeros(nz)
    for k in range(N):
        q_vec[k * 2] = -s.QP_COST_FACTOR * s.Q_POS * delta_seq[k + 1][0]
        q_vec[k * 2 + 1] = -s.QP_COST_FACTOR * s.Q_VEL * delta_seq[k + 1][1]
    for k in range(N):
        rhs = delta_seq[k + 1] - Ad @ delta_seq[k]
        if k == 0:
            rhs = rhs + Ad @ anchor_error
        l[k * 2:(k + 1) * 2] = rhs
        u[k * 2:(k + 1) * 2] = rhs
    l[2 * N:] = -s.DTAU_MAX
    u[2 * N:] = s.DTAU_MAX
    return q_vec, l, u


def error_clamp_limits(s, M_design, mpc_dt, n_joints):
    """按 Settings 取钳位上限。实现在 dyn_model，gen_code.py 调同一个函数。"""
    return dyn.clamp_limits(M_design, s.N, s.Q_POS, s.Q_VEL, s.R_WEIGHT,
                            mpc_dt, s.DTAU_MAX, s.E_CLAMP_FACTOR)


def _make_solvers(s, P, Ad_list, Bd_list, N, nz, n_con, n_joints):
    """每关节一个持久化 OSQP 实例，Ad/Bd 固定，只在线更新 q/l/u。"""
    solvers = []
    for j in range(n_joints):
        A_qp = build_qp_matrix(Ad_list[j], Bd_list[j], N, nz, n_con)
        solver = osqp.OSQP()
        solver.setup(P, np.zeros(nz), A_qp, np.zeros(n_con), np.zeros(n_con),
                     rho=s.OSQP_RHO,
                     eps_abs=s.OSQP_EPS_ABS,
                     eps_rel=s.OSQP_EPS_REL,
                     max_iter=s.OSQP_MAX_ITER,
                     check_termination=s.OSQP_CHECK_TERMINATION,
                     verbose=False,
                     warm_start=s.OSQP_WARM_START,
                     adaptive_rho=False,
                     polish=False)
        solvers.append(solver)
    return solvers


def run(overrides=None, progress=False, debug_steps=0, qp_record_steps=()):
    """跑一次闭环仿真，返回日志和元信息的字典。

    progress         打印进度和扰动附近的详细调试
    debug_steps      头几步打印 OSQP 细节
    qp_record_steps  在这些步记录 QP 的输入输出，供 C 与 Python 数值回归使用
    """
    s = Settings(overrides)
    model, data = load_model(s.URDF_PATH)
    n_joints = model.nv
    if n_joints != 6:
        raise ValueError(f"current simulation requires a 6-axis URDF, got {n_joints}")

    dt = s.BASE_DT
    n_steps = int(s.T_SIM / dt)
    perturb_step = int(s.PERTURB_T / dt)
    perturb_dq = np.array(s.PERTURB_DQ, dtype=float)
    mpc_dt = resolve_mpc_dt(s, n_joints)
    get_reference, q_init, q_goal = make_reference(s, n_joints)

    N = s.N
    nz = 3 * N
    n_con = 3 * N
    dtau_max = s.DTAU_MAX

    M_design, inertia_info = design_inertia(s, model, data)
    if progress:
        print(f"关节数: {n_joints}")
        dyn.print_inertia_report(inertia_info)

    # 重力梯度项：完整 RNEA 前馈已补偿名义重力时默认不重复加入。
    zero = np.zeros(n_joints)
    G0 = pin.rnea(model, data, np.zeros(n_joints), zero, zero)
    eps = 1e-6
    dG_dq = np.zeros(n_joints)
    for j in range(n_joints):
        qp_ = np.zeros(n_joints)
        qp_[j] += eps
        dG_dq[j] = (pin.rnea(model, data, qp_, zero, zero)[j] - G0[j]) / eps

    Ad_list, Bd_list = [], []
    for j in range(n_joints):
        a = dG_dq[j] / M_design[j] if s.USE_GRAVITY_GRADIENT_MODEL else 0.0
        Ac = np.array([[0.0, 1.0], [-a, 0.0]])
        Bc = np.array([[0.0], [-1.0 / M_design[j]]])
        Ad_list.append(np.eye(2) + Ac * mpc_dt)
        Bd_list.append(Bc * mpc_dt)

    e_max, de_max, K_pos, K_vel = error_clamp_limits(s, M_design, mpc_dt, n_joints)
    P = build_p_matrix(s, N, nz)
    solvers = _make_solvers(s, P, Ad_list, Bd_list, N, nz, n_con, n_joints)

    q = q_init.copy()
    dq = np.zeros(n_joints)
    dtau_feedback = np.zeros(n_joints)

    log = {k: [] for k in ("t", "q", "dq", "q_ref", "dq_ref", "e", "e_dq",
                           "tau", "tau_ff", "dtau")}
    # 求解日志用 solve_ 前缀，避免和按步记录的状态日志键名混淆。
    solve_log = {k: [] for k in ("solve_step", "solve_joint", "solve_iter",
                                 "solve_status", "solve_time_us",
                                 "solve_clamped", "solve_unconverged")}
    qp_records = []
    record_at = set(int(x) for x in qp_record_steps)

    if progress:
        print(f"sim: 锚点误差 MPC + 持久化 OSQP + 错峰调度开始...")
        print(f"     基础周期={dt*1e3:.1f} ms, 每周期求解={s.JOINTS_PER_TICK} 个关节, "
              f"单关节更新周期={mpc_dt*1e3:.1f} ms")

    for step in range(n_steps):
        t = step * dt
        q_ref, dq_ref, ddq_ref = get_reference(t)
        tau_ff = pin.rnea(model, data, q, dq, ddq_ref)
        e_q = q_ref - q
        e_dq = dq_ref - dq

        first = (step * s.JOINTS_PER_TICK) % n_joints
        for off in range(s.JOINTS_PER_TICK):
            j = (first + off) % n_joints
            Ad = Ad_list[j]

            delta_seq = np.zeros((N + 1, 2))
            for k in range(1, N + 1):
                qr_k, dr_k, _ = get_reference(t + k * mpc_dt)
                delta_seq[k, 0] = q_ref[j] - qr_k[j]
                delta_seq[k, 1] = dq_ref[j] - dr_k[j]

            anchor_raw = np.array([e_q[j], e_dq[j]])
            if e_max is None:
                anchor = anchor_raw
                clamped = False
            else:
                anchor = np.array([np.clip(anchor_raw[0], -e_max[j], e_max[j]),
                                   np.clip(anchor_raw[1], -de_max[j], de_max[j])])
                clamped = bool(np.any(anchor != anchor_raw))

            qv, l, u = build_vectors(s, N, nz, n_con, Ad, anchor, delta_seq)
            solver = solvers[j]
            solver.update(q=qv, l=l, u=u)
            t0 = time.perf_counter()
            res = solver.solve()
            time_us = (time.perf_counter() - t0) * 1e6

            usable = res.x is not None and np.all(np.isfinite(res.x))
            converged = res.info.status_val in (1, 2)
            # max_iter 用尽但迭代点可用时，方向通常已正确，比 hold 过期值更好。
            accept = usable and (converged or s.USE_UNCONVERGED_ITERATE)
            if accept:
                dtau_feedback[j] = float(np.clip(res.x[2 * N], -dtau_max, dtau_max))

            solve_log["solve_step"].append(step)
            solve_log["solve_joint"].append(j)
            solve_log["solve_iter"].append(res.info.iter)
            solve_log["solve_status"].append(res.info.status_val)
            solve_log["solve_time_us"].append(time_us)
            solve_log["solve_clamped"].append(clamped)
            solve_log["solve_unconverged"].append(not converged)

            if step in record_at:
                qp_records.append({
                    "step": step, "joint": j,
                    "anchor_raw": anchor_raw.copy(), "anchor": anchor.copy(),
                    "delta_seq": delta_seq.copy(),
                    "q_vec": qv.copy(), "l": l.copy(), "u": u.copy(),
                    "dtau": dtau_feedback[j],
                    "iter": res.info.iter, "status": res.info.status_val,
                })

            if progress and (step < debug_steps
                             or perturb_step + 1 <= step <= perturb_step + 4):
                tag = "DEBUG" if step < debug_steps else "PERTURB"
                x = res.x if res.x is not None else np.zeros(nz)
                print(f"\n[{tag} step={step} joint={j}] status={res.info.status} "
                      f"iter={res.info.iter}")
                print(f"  anchor_raw = {anchor_raw}   -> anchor = {anchor}"
                      f"{'  (钳位)' if clamped else ''}")
                print(f"  x[2N]      = {x[2*N]:+.6e}   ||u_full|| = "
                      f"{np.linalg.norm(x[2*N:]):.6e}")

        dtau = dtau_feedback.copy()
        tau = tau_ff + dtau
        ddq = pin.aba(model, data, q, dq, tau)

        # 扰动注入：本步的 aba 已经算完，下一步的 aba 才会看到突变。
        if step == perturb_step:
            q = q + perturb_dq

        dq = dq + ddq * dt
        q = q + dq * dt

        for key, val in (("t", t), ("q", q), ("dq", dq), ("q_ref", q_ref),
                         ("dq_ref", dq_ref), ("e", e_q), ("e_dq", e_dq),
                         ("tau", tau), ("tau_ff", tau_ff), ("dtau", dtau)):
            log[key].append(val.copy() if isinstance(val, np.ndarray) else val)

        if progress and step % 5000 == 0:
            print(f"  step {step:5d}: t={t:.3f}s  ||e_q||={np.linalg.norm(e_q):.5f}  "
                  f"||tau_ff||={np.linalg.norm(tau_ff):.3f}  "
                  f"||dtau||={np.linalg.norm(dtau):.4f}")

    out = {k: np.array(v) for k, v in log.items()}
    out.update({k: np.array(v) for k, v in solve_log.items()})
    out.update({
        "settings": s, "n_joints": n_joints, "dt": dt, "mpc_dt": mpc_dt,
        "n_steps": n_steps, "perturb_step": perturb_step,
        "perturb_dq": perturb_dq, "M_design": M_design,
        "inertia_info": inertia_info, "K_pos": K_pos, "K_vel": K_vel,
        "e_max": e_max, "de_max": de_max, "qp_records": qp_records,
    })
    return out


SETTLE_TOL = 0.01          # 判定"已恢复"的位置误差阈值 [rad]


def _safe_absmax(a):
    """空切片返回 nan，方便用很短的 T_SIM 做冒烟测试而不炸在统计里。"""
    return float(np.abs(a).max()) if np.asarray(a).size else float("nan")


def summarize(res, settle_tol=SETTLE_TOL):
    """从 run() 的结果算出统计量。sim.py 和 sweep_rt.py 用同一套定义。"""
    s = res["settings"]
    dt = res["dt"]
    n_joints = res["n_joints"]
    pstep = res["perturb_step"]
    log_e, log_dtau = res["e"], res["dtau"]
    n_log = log_e.shape[0]

    traj_end = min(int(s.T_TRAJ / dt), n_log)
    ss_start = min(int((s.T_TRAJ + 1.0) / dt), n_log)   # 轨迹结束 1 s 后算静止

    per_joint = []
    for j in range(n_joints):
        post = np.abs(log_e[pstep:, j])
        bad = np.nonzero(post > settle_tol)[0]
        if bad.size == 0:
            settle = 0.0
        elif bad[-1] + 1 >= post.size:
            settle = float("nan")              # 仿真结束时仍未恢复
        else:
            settle = (bad[-1] + 1) * dt
        per_joint.append({
            "e_traj": _safe_absmax(log_e[:traj_end, j]),
            "e_pert": _safe_absmax(post),
            "e_ss": _safe_absmax(log_e[ss_start:pstep, j]),
            "tau_max": _safe_absmax(res["tau"][:, j]),
            "ff_max": _safe_absmax(res["tau_ff"][:, j]),
            "fb_max": _safe_absmax(log_dtau[:, j]),
            "sat_pct": 100.0 * float(np.mean(
                np.abs(log_dtau[:, j]) >= s.DTAU_MAX * 0.999)),
            "settle": settle,
        })

    it = res["solve_iter"]
    st = res["solve_step"]
    phases = [
        ("轨迹段", 0, int(s.T_TRAJ / dt)),
        ("扰动前静止", int(s.T_TRAJ / dt), pstep),
        ("扰动瞬态.3s", pstep, pstep + int(0.3 / dt)),
        ("扰动后稳态", pstep + int(0.3 / dt), res["n_steps"]),
    ]
    phase_stats = []
    for name, lo, hi in phases:
        seg = it[(st >= lo) & (st < hi)]
        if seg.size == 0:
            continue
        phase_stats.append({
            "name": name, "n": int(seg.size), "mean": float(seg.mean()),
            "max": int(seg.max()), "p99": float(np.percentile(seg, 99)),
        })

    settles = np.array([p["settle"] for p in per_joint])
    return {
        "per_joint": per_joint,
        "phases": phase_stats,
        "n_solves": int(it.size),
        "iter_mean": float(it.mean()),
        "iter_max": int(it.max()),
        "iter_p99": float(np.percentile(it, 99)),
        "time_mean_us": float(res["solve_time_us"].mean()),
        "time_p99_us": float(np.percentile(res["solve_time_us"], 99)),
        "n_solved": int(np.sum(res["solve_status"] == 1)),
        "n_inaccurate": int(np.sum(res["solve_status"] == 2)),
        "n_other": int(np.sum(~np.isin(res["solve_status"], (1, 2)))),
        "n_unconverged": int(np.sum(res["solve_unconverged"])),
        "clamp_pct": 100.0 * float(np.mean(res["solve_clamped"])),
        "e_traj_max": float(np.nanmax([p["e_traj"] for p in per_joint])),
        "e_ss_max": float(np.nanmax([p["e_ss"] for p in per_joint])),
        "sat_pct_max": float(np.nanmax([p["sat_pct"] for p in per_joint])),
        "settle_max": float(np.nanmax(settles)) if np.any(~np.isnan(settles)) else float("nan"),
        "n_not_settled": int(np.sum(np.isnan(settles))),
    }


def print_summary(res, summary, settle_tol=SETTLE_TOL):
    """完整统计打印。单次运行用；扫描时只看 sweep_rt.py 的汇总表。"""
    s = res["settings"]
    n_joints = res["n_joints"]

    print("\n========== 每关节精度 ==========")
    print(f"{'j':>3} {'e轨迹':>10} {'e扰动':>10} {'e稳态':>10} {'max|tau|':>10} "
          f"{'max|ff|':>10} {'max|fb|':>10} {'饱和%':>8} {'恢复s':>8}")
    for j, p in enumerate(summary["per_joint"]):
        print(f"{j+1:>3} {p['e_traj']:>10.5f} {p['e_pert']:>10.5f} "
              f"{p['e_ss']:>10.5f} {p['tau_max']:>10.3f} {p['ff_max']:>10.3f} "
              f"{p['fb_max']:>10.3f} {p['sat_pct']:>8.2f} {p['settle']:>8.3f}")
    print(f"(e轨迹 = t<{s.T_TRAJ:g}s 运动段; e扰动 = t>={s.PERTURB_T:g}s 扰动段; "
          f"e稳态 = 扰动前静止段)")
    print(f"(饱和% = |dtau| 触及 DTAU_MAX={s.DTAU_MAX:g} Nm 的占比; "
          f"恢复s = 扰动后 |e| 回到 {settle_tol} rad 内所需时间, nan=未恢复)")

    print("\n========== 残差模型增益 ==========")
    print(f"{'j':>3} {'M_eff':>10} {'K_pos':>10} {'K_vel':>10} "
          f"{'rho':>9} {'ts':>8} {'e_max':>9} {'de_max':>9}")
    for j in range(n_joints):
        rho, ts = dyn.closed_loop_pole(res["M_design"][j], res["K_pos"][j],
                                       res["K_vel"][j], res["mpc_dt"])
        em = "-" if res["e_max"] is None else f"{res['e_max'][j]:.4f}"
        dm = "-" if res["de_max"] is None else f"{res['de_max'][j]:.4f}"
        print(f"{j+1:>3} {res['M_design'][j]:>10.5f} {res['K_pos'][j]:>10.2f} "
              f"{res['K_vel'][j]:>10.2f} {rho:>9.4f} {ts:>8.3f} {em:>9} {dm:>9}")
    print("(rho/ts 按设计惯量算，即最坏构型；其他构型惯量更大、响应更慢但更稳)")

    print("\n========== OSQP 参数 ==========")
    for name in ("OSQP_RHO", "OSQP_EPS_ABS", "OSQP_EPS_REL", "OSQP_MAX_ITER",
                 "OSQP_CHECK_TERMINATION", "OSQP_WARM_START",
                 "E_CLAMP_FACTOR", "USE_UNCONVERGED_ITERATE"):
        print(f"  {name:<24}= {getattr(s, name)}")

    print("\n========== 求解器总体性能 ==========")
    print(f"总求解次数:   {summary['n_solves']}")
    print(f"status 分布:  1(solved)={summary['n_solved']}, "
          f"2(inaccurate)={summary['n_inaccurate']}, 其他={summary['n_other']}")
    print(f"未收敛次数:   {summary['n_unconverged']} "
          f"(处置: {'用当前迭代点' if s.USE_UNCONVERGED_ITERATE else 'hold 上周期'})")
    print(f"误差钳位占比: {summary['clamp_pct']:.2f}%")
    print(f"迭代次数:     mean={summary['iter_mean']:.1f}, "
          f"max={summary['iter_max']}, p99={summary['iter_p99']:.0f}")
    print(f"求解耗时(us): mean={summary['time_mean_us']:.1f}, "
          f"p99={summary['time_p99_us']:.1f}  (PC 数值, 对 MCU 无参考意义)")

    print("\n========== 迭代数分阶段 ==========")
    print(f"{'阶段':>12} {'次数':>7} {'iter_mean':>10} {'iter_max':>9} {'iter_p99':>9}")
    for ph in summary["phases"]:
        print(f"{ph['name']:>12} {ph['n']:>7} {ph['mean']:>10.1f} "
              f"{ph['max']:>9} {ph['p99']:>9.0f}")
    print(f"(max_iter={s.OSQP_MAX_ITER}; H7@480MHz 单次迭代约 1~3 us, "
          f"1 ms 预算内目标 iter_max <= 100)")

    print("\n========== 每关节求解器性能 ==========")
    print(f"{'j':>3} {'次数':>7} {'iter_mean':>10} {'iter_max':>9} {'钳位%':>8}")
    for j in range(n_joints):
        m = res["solve_joint"] == j
        print(f"{j+1:>3} {int(m.sum()):>7} {res['solve_iter'][m].mean():>10.1f} "
              f"{int(res['solve_iter'][m].max()):>9} "
              f"{100.0*res['solve_clamped'][m].mean():>8.2f}")
