"""闭环仿真内核。sim.py（单次运行 + 画图）和 sweep_rt.py（参数扫描）共用。

锚点误差 MPC + 参考偏移 δ + 错峰 + 每关节独立 solver + 热启动，
与嵌入式版本一一对应。

run() 接受 overrides 覆盖 mpc_config 里的任意大写配置项，不会污染 cfg 模块，
所以一个进程里可以连续跑多组配置。模型加载和惯量扫描按 key 缓存，扫描时不重复算。
"""
import collections
import time

import numpy as np
import osqp
import pinocchio as pin
from scipy import sparse

import mpc_config as cfg
import dyn_model as dyn
import cmpc

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


def load_model(urdf_path, armature=None):
    """pinocchio 模型按 (路径, armature) 缓存。data 每次新建，避免跨 run 共享可变状态。

    armature 是折算到输出端的关节电机转子惯量，加在惯量矩阵对角上。
    pinocchio 的 rnea / crba / aba 都会计入它，所以前馈和残差模型自动一致。
    """
    arm_key = None if armature is None else tuple(float(v) for v in armature)
    key = (urdf_path, arm_key)
    if key not in _MODEL_CACHE:
        model = pin.buildModelFromUrdf(urdf_path)
        if arm_key is not None:
            arm = np.asarray(arm_key, dtype=float)
            if arm.size != model.nv:
                raise ValueError(
                    f"ARMATURE 长度必须等于关节数 {model.nv}，got {arm.size}")
            if np.any(arm < 0.0) or not np.all(np.isfinite(arm)):
                raise ValueError(f"ARMATURE 必须是非负有限值，got {arm}")
            model.armature = arm
        _MODEL_CACHE[key] = model
    model = _MODEL_CACHE[key]
    return model, model.createData()


def build_plant(s, nominal):
    """被控对象模型 = 标称模型 + 质量偏差（+ 负载另算一份）。

    控制器用标称模型算前馈，被控对象用这一份积分，两者的差就是模型误差。
    返回 (plant_no_payload, plant_with_payload)；没配负载时第二项是 None。

    注意 armature 不参与质量缩放：那是电机转子，不随连杆质量属性变。
    """
    plant = nominal.copy()
    scale = float(s.PLANT_MASS_SCALE)
    if scale <= 0.0 or not np.isfinite(scale):
        raise ValueError(f"PLANT_MASS_SCALE 必须为正有限值，got {scale}")
    if scale != 1.0:
        for i in range(1, plant.njoints):
            ine = plant.inertias[i]
            plant.inertias[i] = pin.Inertia(
                ine.mass * scale, ine.lever, ine.inertia * scale)

    payload_mass = float(s.PAYLOAD_MASS)
    if payload_mass <= 0.0:
        return plant, None
    loaded = plant.copy()
    com = np.asarray(s.PAYLOAD_COM, dtype=float)
    if com.size != 3:
        raise ValueError("PAYLOAD_COM 必须是长度 3 的向量")
    # 点质量：绕自身质心的惯量为 0，平移到连杆系由 Inertia 自己处理
    tip = loaded.njoints - 1
    loaded.inertias[tip] = loaded.inertias[tip] + pin.Inertia(
        payload_mass, com, np.zeros((3, 3)))
    return plant, loaded


# 被控对象侧库伦摩擦在零速附近的过渡宽度 [rad/s]。
#
# 这里有过一个数值伪影：原来取 1e-3、对象和控制器共用 2 ms 显式积分。
# 库伦摩擦在零速附近相当于一个阻尼 Fc/eps，J5 是 0.08/1e-3 = 80 Nm·s/rad，
# 除以惯量 0.0026 得到衰减率约 3e4 1/s，显式 Euler 稳定要求 h < 2/3e4 = 65 µs，
# 2 ms 远远超了——对象的摩擦力每一步都在翻符号，静止时 J5 的速度
# 白白抖到 0.06~0.2 rad/s（关掉传感器噪声照样抖，这就是证据）。
# 现在对象走 PLANT_SUBSTEPS 个子步长，eps 放宽到 5e-3，两者一起保证稳定，
# 启动时会检查并报出来。
PLANT_FRICTION_DQ_EPS = 5.0e-3


def plant_substeps(s, dt, m_min):
    """被控对象每个控制周期的子步长数，PLANT_SUBSTEPS 为 None 时自动算。"""
    if s.PLANT_SUBSTEPS is not None:
        n = int(s.PLANT_SUBSTEPS)
        if n < 1:
            raise ValueError("PLANT_SUBSTEPS 必须 >= 1")
        return n
    fc = np.asarray(s.FRICTION_COULOMB, dtype=float)
    fv = np.asarray(s.FRICTION_VISCOUS, dtype=float)
    lam = float(np.max((fc / PLANT_FRICTION_DQ_EPS + fv) / np.asarray(m_min)))
    return max(1, int(np.ceil(dt * lam)))


def friction_torque(s, dq):
    """关节摩擦力矩（库伦 + 粘性），符号与运动方向相反。

    只有库伦 + 粘性两项。**没有**静摩擦/Stribeck 段、没有随位置变化的
    齿轮啮合波动、没有温度相关性，所以低速爬行(stick-slip)在这里看不到。
    """
    cou = np.asarray(s.FRICTION_COULOMB, dtype=float)
    vis = np.asarray(s.FRICTION_VISCOUS, dtype=float)
    return -(cou * np.tanh(dq / PLANT_FRICTION_DQ_EPS) + vis * dq)


def measure(s, q, dq, rng):
    """反馈量化 + 噪声。返回控制器实际看到的 (q_meas, dq_meas)。"""
    if not s.FEEDBACK_NOISE:
        return q, dq
    q_lsb = np.asarray(s.NOISE_Q_LSB, dtype=float)
    dq_lsb = np.asarray(s.NOISE_DQ_LSB, dtype=float)
    # 先量化到 LSB 网格，再叠加 ±0.5 LSB 的均匀噪声：
    # 量化代表编码器分辨率，额外那层噪声代表抖动和传输时刻差异。
    # 注意这是**量化 + 额外抖动**，总误差可达 1 LSB，比纯量化(±0.5 LSB)
    # 保守一倍；真实达妙的速度反馈是内部差分出来的，噪声特性未必是均匀分布。
    qm = np.round(q / q_lsb) * q_lsb + (rng.random(q.size) - 0.5) * q_lsb
    dqm = np.round(dq / dq_lsb) * dq_lsb + (rng.random(dq.size) - 0.5) * dq_lsb
    return qm, dqm


def design_inertia(s, model, data):
    """惯量选型结果按影响它的配置项缓存。"""
    key = (s.URDF_PATH, tuple(float(v) for v in s.ARMATURE),
           s.INERTIA_MODE, s.INERTIA_SAMPLES, s.INERTIA_SEED,
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


control_bounds = dyn.control_bounds      # 实现在 dyn_model，gen_code 调同一份


def build_vectors(s, N, nz, n_con, Ad, anchor_error, delta_seq,
                  dtau_max_j, effort_j, tau_ff_j, rho_j=1.0, w_hat_j=0.0):
    """在线 QP 向量。与生成的 C 版 mpc_build_vectors() 逐行对应。

    dtau_max_j / effort_j / tau_ff_j 一起决定 Δτ 的箱约束（见 control_bounds），
    rho_j 把它换算到决策变量 u 上（Δτ = ρ·u，P3），
    w_hat_j 是观测器估计的扰动加速度，作为已知常值写进预测模型（P1）：
        de⁺ = de - DT·(u/M_design + ŵ)   ->  每一步等式右端加 c = [0, -DT·ŵ]

    锚点误差 e_anchor = x_ref(t0) - x(t)，与普通跟踪误差不同。
    由 r_k = e_anchor,k - delta_k 得 e_(k+1) = A*e_k + B*u_k + c + delta_(k+1) - A*delta_k，
    第一步还要把当前锚点误差 A*e_0 移到等式右端。
    """
    l = np.zeros(n_con)
    u = np.zeros(n_con)
    q_vec = np.zeros(nz)
    for k in range(N):
        q_vec[k * 2] = -s.QP_COST_FACTOR * s.Q_POS * delta_seq[k + 1][0]
        q_vec[k * 2 + 1] = -s.QP_COST_FACTOR * s.Q_VEL * delta_seq[k + 1][1]
    c1 = -Ad[0, 1] * w_hat_j               # Ad[0,1] 就是 MPC_DT
    for k in range(N):
        rhs = delta_seq[k + 1] - Ad @ delta_seq[k]
        if k == 0:
            rhs = rhs + Ad @ anchor_error
        rhs = rhs + np.array([0.0, c1])
        l[k * 2:(k + 1) * 2] = rhs
        u[k * 2:(k + 1) * 2] = rhs
    lo, hi = dyn.control_bounds_u(dtau_max_j, effort_j, tau_ff_j, rho_j)
    l[2 * N:] = lo
    u[2 * N:] = hi
    return q_vec, l, u


def error_clamp_limits(s, M_design, mpc_dt, n_joints):
    """按 Settings 取钳位上限。实现在 dyn_model，gen_code.py 调同一个函数。"""
    return dyn.clamp_limits(M_design, s.N, s.Q_POS, s.Q_VEL, s.R_WEIGHT,
                            mpc_dt, dtau_max_array(s, n_joints),
                            s.E_CLAMP_FACTOR)


def dtau_max_array(s, n_joints):
    """DTAU_MAX 规范化成长度为关节数的数组，标量会被广播（兼容旧配置）。"""
    v = np.asarray(s.DTAU_MAX, dtype=float)
    if v.ndim == 0:
        v = np.full(n_joints, float(v))
    if v.size != n_joints:
        raise ValueError(f"DTAU_MAX 长度必须等于关节数 {n_joints}，got {v.size}")
    if np.any(v <= 0.0) or not np.all(np.isfinite(v)):
        raise ValueError(f"DTAU_MAX 必须是正有限值，got {v}")
    return v


effort_array = dyn.effort_array          # 同上，实现在 dyn_model
friction_feedforward = dyn.friction_feedforward


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
    model, data = load_model(s.URDF_PATH, s.ARMATURE)
    n_joints = model.nv
    if n_joints != 6:
        raise ValueError(f"current simulation requires a 6-axis URDF, got {n_joints}")

    dt = s.BASE_DT
    n_steps = int(s.T_SIM / dt)
    perturb_step = int(s.PERTURB_T / dt)
    perturb_dq = np.array(s.PERTURB_Q_STEP, dtype=float)

    # 被控对象：标称模型 + 质量偏差，另备一份挂了负载的
    plant, plant_loaded = build_plant(s, model)
    plant_data = plant.createData()
    loaded_data = None if plant_loaded is None else plant_loaded.createData()
    payload_step = (int(s.PAYLOAD_T / dt) if plant_loaded is not None
                    else n_steps + 1)
    impact_step = int(s.IMPACT_T / dt)
    impact_steps = max(1, int(round(s.IMPACT_DURATION / dt)))
    impact_tau = np.asarray(s.IMPACT_TAU, dtype=float)
    rng = np.random.default_rng(s.NOISE_SEED)

    # 被控对象的子步长数。零速附近的库伦摩擦等效阻尼 fc/eps + fv，
    # 除以最小关节惯量就是最快的衰减率 λ，显式 Euler 要 h·λ <= 1 才不会来回翻。
    n_sub = plant_substeps(s, dt, design_inertia(s, model, data)[0])

    # 执行器延迟：本周期算出的力矩要等 delay 个 tick 之后才真正作用到对象上
    delay = int(s.CTRL_DELAY_TICKS)
    if delay < 0:
        raise ValueError("CTRL_DELAY_TICKS 必须 >= 0")
    tau_pipe = collections.deque([np.zeros(n_joints)] * delay, maxlen=delay + 1)

    # 反馈延迟：控制器这一拍读到的是 fb_delay 个 tick 之前的反馈。
    # 实物上 CAN 反馈在中断里写进 measure，robottask 下一次运行才读，
    # 所以和执行侧一样有 0~1 个 tick 的年龄。管道在下面状态初始化后再建。
    fb_delay = int(s.FEEDBACK_DELAY_TICKS)
    if fb_delay < 0:
        raise ValueError("FEEDBACK_DELAY_TICKS 必须 >= 0")
    mpc_dt = resolve_mpc_dt(s, n_joints)
    get_reference, q_init, q_goal = make_reference(s, n_joints)

    N = s.N
    nz = 3 * N
    n_con = 3 * N
    dtau_max = dtau_max_array(s, n_joints)
    effort = effort_array(s, model)

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

    if s.SOLVER not in ("cs_admm", "osqp"):
        raise ValueError(f"SOLVER 只能是 'cs_admm' 或 'osqp'，got {s.SOLVER!r}")
    use_cs = s.SOLVER == "cs_admm"
    if use_cs:
        # P7: 消元 + 定迭代次数 ADMM；P4: 每拍解全部关节，预测步长 = BASE_DT
        cs = cmpc.design(s, M_design)
        cs_solver = cmpc.Solver(cs)
        K_pos, K_vel = cs["K0"][:, 0], cs["K0"][:, 1]
        e_max, de_max = cmpc.clamp_limits(s, cs, dtau_max_array(s, n_joints))
        q_lo, q_hi, v_hi = cmpc.joint_limits(s, model)
        mpc_dt = dt
        solvers = None
    else:
        cs = cs_solver = None
        e_max, de_max, K_pos, K_vel = error_clamp_limits(s, M_design, mpc_dt, n_joints)
        P = build_p_matrix(s, N, nz)
        solvers = _make_solvers(s, P, Ad_list, Bd_list, N, nz, n_con, n_joints)

    # ---- P3: 惯量比 ----
    if s.MPC_FORM not in ("ddq", "tau"):
        raise ValueError(f"MPC_FORM 只能是 'ddq' 或 'tau'，got {s.MPC_FORM!r}")
    use_rho = s.MPC_FORM == "ddq"
    rho_lo, rho_hi = dyn.inertia_ratio_limits(s, inertia_info)

    # ---- P1 + P2: 观测器和延迟补偿 ----
    if s.ESTIMATOR not in ("kalman", "none"):
        raise ValueError(f"ESTIMATOR 只能是 'kalman' 或 'none'，got {s.ESTIMATOR!r}")
    use_est = s.ESTIMATOR == "kalman"
    # 不开观测器时保持旧行为：所有参考都取"现在"，不做任何时间对齐
    est_fb = int(s.EST_FB_DELAY_TICKS) if (use_est and s.DELAY_COMP) else 0
    est_ct = int(s.EST_CTRL_DELAY_TICKS) if (use_est and s.DELAY_COMP) else 0
    if est_fb < 0 or est_ct < 0:
        raise ValueError("EST_*_DELAY_TICKS 必须 >= 0")
    use_wcomp = dyn.dist_comp_mask(s, n_joints)
    if use_est:
        obs_F, obs_G, obs_L = dyn.observer_design(s, M_design)
        est = dyn.Estimator(obs_F, obs_G, obs_L, est_fb + est_ct)
    else:
        obs_F = obs_G = obs_L = None
        est = None

    q = q_init.copy()
    dq = np.zeros(n_joints)
    u_hold = np.zeros(n_joints)    # QP 的决策变量 u，没轮到的关节沿用
    n_tau_saturated = 0            # 总力矩被 effort 截掉的关节-周期数
    fb_pipe = collections.deque(
        [(q_init.copy(), np.zeros(n_joints))] * fb_delay, maxlen=fb_delay + 1)

    # e 是控制器看到的误差（含噪声），e_true 是真实误差——评价精度要看后者。
    # q_meas/dq_meas/ddq_ref 是控制器的全部输入，ctest/dump_loop.py 要靠它们
    # 在 C 里重放整个 tick 循环。
    # qr_m/dqr_m 是测量时刻的参考（观测器用），qr_a/dqr_a/ddqr_a 是力矩生效时刻的
    # 参考（前馈和 MPC 用），rho 是惯量比，w_hat 是扰动估计。
    # est_z/est_hist 是每拍更新**之前**的观测器状态，C 回归从它热启动。
    log = {k: [] for k in ("t", "q", "dq", "q_ref", "dq_ref", "e", "e_dq",
                           "tau", "tau_ff", "dtau", "tau_dist", "e_true",
                           "q_meas", "dq_meas", "ddq_ref",
                           "qr_m", "dqr_m", "qr_a", "dqr_a", "ddqr_a",
                           "rho", "w_hat", "est_z", "est_hist",
                           "qr_end", "dqr_end", "ddqr_end", "qr_e", "dqr_e")}
    # 求解日志用 solve_ 前缀，避免和按步记录的状态日志键名混淆。
    solve_log = {k: [] for k in ("solve_step", "solve_joint", "solve_iter",
                                 "solve_status", "solve_time_us",
                                 "solve_clamped", "solve_unconverged")}
    qp_records = []
    record_at = set(int(x) for x in qp_record_steps)

    if progress and use_cs:
        print(f"sim: 消元 + 定迭代 ADMM（{cs['iters']} 次），每拍解全部 {n_joints} 个关节，"
              f"时域 {cs['NS']} 拍 = {cs['NS']*dt*1e3:g} ms，带位置/速度约束...")
    elif progress:
        print(f"sim: 锚点误差 MPC + 持久化 OSQP + 错峰调度开始...")
        print(f"     基础周期={dt*1e3:.1f} ms, 每周期求解={s.JOINTS_PER_TICK} 个关节, "
              f"单关节更新周期={mpc_dt*1e3:.1f} ms")

    for step in range(n_steps):
        t = step * dt
        q_ref, dq_ref, ddq_ref = get_reference(t)
        # 两个时刻的参考：测量是 est_fb 拍前的，力矩要 est_ct 拍后才生效。
        # 轨迹是"可求值计划"，任意时刻都能求，这一点板上的 ins_traj 也满足。
        t_meas = max(0.0, t - est_fb * dt)
        t_apply = t + est_ct * dt
        qr_m, dqr_m, _ = get_reference(t_meas)
        qr_a, dqr_a, ddqr_a = get_reference(t_apply)
        # 控制器只能看到带噪声、量化过的反馈，前馈和误差都用它
        q_meas, dq_meas = measure(s, q, dq, rng)
        # 反馈延迟：控制器看到的是 fb_delay 拍之前的测量值
        if fb_delay > 0:
            fb_pipe.append((q_meas, dq_meas))
            q_meas, dq_meas = fb_pipe[0]
        # 前馈 = RNEA + 摩擦补偿。用力矩生效时刻的参考加速度（P2）。
        # 摩擦补偿也算前馈的一部分，所以它要进总力矩约束，否则 QP 以为还有余量。
        tau_ff = (np.asarray(pin.rnea(model, data, q_meas, dq_meas, ddqr_a))
                  + friction_feedforward(s, dq_meas))
        e_q = q_ref - q_meas
        e_dq = dq_ref - dq_meas

        # P3: 当前构型的惯量比
        if use_rho:
            rho = dyn.inertia_ratio(model, data, q_meas, M_design, rho_lo, rho_hi)
        else:
            rho = np.ones(n_joints)

        # P1: 观测器在测量时刻更新，每拍所有关节都更新（不管这拍解不解它的 QP）
        if use_est:
            est_z_before = est.z.copy()
            est_hist_before = est.hist.copy()
            for j in range(n_joints):
                est.update(j, qr_m[j] - q_meas[j], dqr_m[j] - dq_meas[j])
            w_hat_all = est.z[:, 2].copy()
        else:
            est_z_before = np.zeros((n_joints, 3))
            est_hist_before = np.zeros((n_joints, 1))
            w_hat_all = np.zeros(n_joints)

        # 参考预览（只有 cs 路径用；osqp 路径记 0 占位，C 回归只测 cs）
        qr_end = dqr_end = ddqr_end = np.zeros(n_joints)
        qr_e = dqr_e = np.zeros((1, n_joints))
        if use_cs:
            # ---- P4/P5/P7: 每拍解全部关节，用新求解器 ----
            # 预测时域里的前馈：生效时刻和时域末尾各算一次参考 RNEA，
            # 差值加到当前前馈上，中间线性插值（cmpc.control_bounds_blocks）。
            # 两次 RNEA 六个关节共用，是每拍新增的主要开销。
            qr_end, dqr_end, ddqr_end = get_reference(t_apply + cs["NS"] * dt)
            tref0 = (np.asarray(pin.rnea(model, data, qr_a, dqr_a, ddqr_a))
                     + friction_feedforward(s, dqr_a))
            tref1 = (np.asarray(pin.rnea(model, data, qr_end, dqr_end, ddqr_end))
                     + friction_feedforward(s, dqr_end))
            tau_ff_end = tau_ff + (tref1 - tref0)
            # P5: 每块末尾时刻的参考（参考预览真正起作用的地方）
            qr_e = np.zeros((cs["NB"], n_joints))
            dqr_e = np.zeros((cs["NB"], n_joints))
            for i, e in enumerate(cs["ends"]):
                qr_e[i], dqr_e[i], _ = get_reference(t_apply + e * dt)
            for j in range(n_joints):
                if use_est:
                    anchor_raw, w_j = est.anchor(j)
                    w_j = w_j if use_wcomp[j] else 0.0
                else:
                    anchor_raw = np.array([e_q[j], e_dq[j]])
                    w_j = 0.0
                if e_max is None:
                    anchor, clamped = anchor_raw, False
                else:
                    anchor = np.array([np.clip(anchor_raw[0], -e_max[j], e_max[j]),
                                       np.clip(anchor_raw[1], -de_max[j], de_max[j])])
                    clamped = bool(np.any(anchor != anchor_raw))
                lo_u, hi_u = cmpc.control_bounds_blocks(
                    cs, dtau_max[j], effort[j], tau_ff[j], tau_ff_end[j], rho[j])
                lo_s, hi_s = cmpc.state_bounds(cs, qr_e[:, j], dqr_e[:, j],
                                               q_lo[j], q_hi[j], v_hi[j])
                if step in record_at:
                    warm = (cs_solver.u[j].copy(), cs_solver.zu[j].copy(),
                            cs_solver.yu[j].copy(), cs_solver.zs[j].copy(),
                            cs_solver.ys[j].copy())
                t0 = time.perf_counter()
                u_hold[j] = cs_solver.solve(j, anchor, anchor_raw, w_j,
                                            lo_u, hi_u, lo_s, hi_s)
                time_us = (time.perf_counter() - t0) * 1e6
                solve_log["solve_step"].append(step)
                solve_log["solve_joint"].append(j)
                solve_log["solve_iter"].append(cs["iters"])
                solve_log["solve_status"].append(1)
                solve_log["solve_time_us"].append(time_us)
                solve_log["solve_clamped"].append(clamped)
                solve_log["solve_unconverged"].append(False)
                if step in record_at:
                    qp_records.append({
                        "step": step, "joint": j, "anchor_raw": anchor_raw.copy(),
                        "anchor": anchor.copy(), "w_hat": float(w_j),
                        "tau_ff": float(tau_ff[j]), "tau_ff_end": float(tau_ff_end[j]),
                        "rho": float(rho[j]), "qr_e": qr_e[:, j].copy(),
                        "dqr_e": dqr_e[:, j].copy(), "warm": warm,
                        "u_sol": u_hold[j], "dtau": rho[j] * u_hold[j],
                    })
        for off in range(0 if use_cs else s.JOINTS_PER_TICK):
            first = (step * s.JOINTS_PER_TICK) % n_joints
            j = (first + off) % n_joints
            Ad = Ad_list[j]

            # 参考预览也以力矩生效时刻为起点
            delta_seq = np.zeros((N + 1, 2))
            for k in range(1, N + 1):
                qr_k, dr_k, _ = get_reference(t_apply + k * mpc_dt)
                delta_seq[k, 0] = qr_a[j] - qr_k[j]
                delta_seq[k, 1] = dqr_a[j] - dr_k[j]

            if use_est:
                # P2: 前推到力矩生效时刻的估计状态当锚点
                anchor_raw, w_j = est.anchor(j)
                w_j = w_j if use_wcomp[j] else 0.0
            else:
                anchor_raw = np.array([e_q[j], e_dq[j]])
                w_j = 0.0
            if e_max is None:
                anchor = anchor_raw
                clamped = False
            else:
                anchor = np.array([np.clip(anchor_raw[0], -e_max[j], e_max[j]),
                                   np.clip(anchor_raw[1], -de_max[j], de_max[j])])
                clamped = bool(np.any(anchor != anchor_raw))

            qv, l, u = build_vectors(s, N, nz, n_con, Ad, anchor, delta_seq,
                                     dtau_max[j], effort[j], tau_ff[j],
                                     rho[j], w_j)
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
                # 取回的迭代点可能越界（未收敛时），按同一组箱约束钳回去，
                # 这样"总力矩不超 effort"在任何情况下都成立
                lo_j, hi_j = dyn.control_bounds_u(dtau_max[j], effort[j],
                                                  tau_ff[j], rho[j])
                u_hold[j] = float(np.clip(res.x[2 * N], lo_j, hi_j))

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
                    "u_sol": u_hold[j],           # QP 的解（u 单位）
                    "dtau": rho[j] * u_hold[j],   # 对应的力矩修正
                    "tau_ff": float(tau_ff[j]),   # C 侧要靠它重建控制量边界
                    "rho": float(rho[j]), "w_hat": float(w_j),
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

        # P3: u 按当前构型的惯量比换算成力矩修正（hold 的关节也用当前的 ρ）
        dtau = rho * u_hold
        # 装配时按 effort 再饱和一次：QP 的约束只在轮到该关节的那一拍成立，
        # hold 期间 tau_ff 变了就可能顶出去（见 dyn.saturate_total 的说明）
        tau, n_sat = dyn.saturate_total(tau_ff, dtau, effort)
        n_tau_saturated += n_sat
        # P1: 告诉观测器这一拍**真正**发出去的修正量（饱和以后反算），
        # 观测器只认实际作用的量，所以积分作用天然不会饱和累积
        if use_est:
            u_applied = (tau - tau_ff) / rho
            for j in range(n_joints):
                est.commit(j, u_applied[j])

        # 执行器延迟：对象这一步受到的是 delay 个 tick 之前算出的力矩
        if delay > 0:
            tau_pipe.append(tau.copy())
            tau_applied = tau_pipe[0]
        else:
            tau_applied = tau

        # ---- 被控对象侧的扰动，控制器都不知道（摩擦和冲击在下面子步里算）----
        # 负载突变：到时刻后换成挂了负载的那份模型
        if step >= payload_step:
            pm, pd = plant_loaded, loaded_data
        else:
            pm, pd = plant, plant_data
        # 被控对象在一个控制周期内走 n_sub 个子步长：力矩是零阶保持的
        # （和实物一样，电机在两帧之间保持上一帧的力矩），摩擦每个子步重算。
        # 控制器仍然 2 ms 一拍，这里只是让对象本身积分得足够细。
        h = dt / n_sub
        for _ in range(n_sub):
            tau_dist = friction_torque(s, dq)
            if impact_step <= step < impact_step + impact_steps:
                tau_dist = tau_dist + impact_tau
            ddq = pin.aba(pm, pd, q, dq, tau_applied + tau_dist)
            dq = dq + ddq * h
            q = q + dq * h

        # 位置瞬移（默认全 0，只用来单独压测求解器）
        if step == perturb_step:
            q = q + perturb_dq

        for key, val in (("t", t), ("q", q), ("dq", dq), ("q_ref", q_ref),
                         ("dq_ref", dq_ref), ("e", e_q), ("e_dq", e_dq),
                         ("tau", tau), ("tau_ff", tau_ff), ("dtau", dtau),
                         ("tau_dist", tau_dist), ("e_true", q_ref - q),
                         ("q_meas", q_meas), ("dq_meas", dq_meas),
                         ("ddq_ref", ddq_ref),
                         ("qr_m", qr_m), ("dqr_m", dqr_m), ("qr_a", qr_a),
                         ("dqr_a", dqr_a), ("ddqr_a", ddqr_a),
                         ("rho", rho), ("w_hat", w_hat_all),
                         ("est_z", est_z_before), ("est_hist", est_hist_before),
                         ("qr_end", qr_end), ("dqr_end", dqr_end),
                         ("ddqr_end", ddqr_end), ("qr_e", qr_e), ("dqr_e", dqr_e)):
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
        "dtau_max": dtau_max, "effort": effort,
        "n_tau_saturated": n_tau_saturated,
        "inertia_info": inertia_info, "K_pos": K_pos, "K_vel": K_vel,
        "e_max": e_max, "de_max": de_max, "qp_records": qp_records,
        "rho_lo": rho_lo, "rho_hi": rho_hi,
        "obs_F": obs_F, "obs_G": obs_G, "obs_L": obs_L,
        "est_D": est_fb + est_ct, "est_fb": est_fb, "est_ct": est_ct,
        "plant_substeps": n_sub, "cs": cs,
        "q_lo": q_lo if use_cs else None, "q_hi": q_hi if use_cs else None,
        "v_hi": v_hi if use_cs else None,
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
    # 精度一律按真实误差 e_true 评价：e 是控制器看到的带噪声版本，
    # 用它评价精度会把测量噪声算成跟踪性能。
    log_e, log_dtau = res["e_true"], res["dtau"]
    n_log = log_e.shape[0]

    def idx(sec):
        return int(min(max(sec, 0.0) / dt, n_log))

    traj_end = idx(s.T_TRAJ)
    ss_start = idx(s.T_TRAJ + 1.0)            # 轨迹结束 1 s 后算静止
    load_step = idx(s.PAYLOAD_T)
    load_ss = idx(s.PAYLOAD_T + 1.0)          # 负载突变 1 s 后算新的稳态
    imp_step = idx(s.IMPACT_T)

    per_joint = []
    for j in range(n_joints):
        # 恢复时间按冲击之后算：|e_true| 最后一次超出容差的时刻
        post = np.abs(log_e[imp_step:, j])
        bad = np.nonzero(post > settle_tol)[0]
        if bad.size == 0:
            settle = 0.0
        elif bad[-1] + 1 >= post.size:
            settle = float("nan")              # 仿真结束时仍未恢复
        else:
            settle = (bad[-1] + 1) * dt
        per_joint.append({
            "e_traj": _safe_absmax(log_e[:traj_end, j]),
            "e_ss": _safe_absmax(log_e[ss_start:load_step, j]),
            "e_load": _safe_absmax(log_e[load_ss:imp_step, j]),
            "e_impact": _safe_absmax(post),
            "tau_max": _safe_absmax(res["tau"][:, j]),
            "ff_max": _safe_absmax(res["tau_ff"][:, j]),
            "fb_max": _safe_absmax(log_dtau[:, j]),
            "dist_max": _safe_absmax(res["tau_dist"][:, j]),
            "sat_pct": 100.0 * float(np.mean(
                np.abs(log_dtau[:, j]) >= res["dtau_max"][j] * 0.999)),
            "settle": settle,
        })

    it = res["solve_iter"]
    st = res["solve_step"]
    phases = [
        ("轨迹段", 0, traj_end),
        ("静止段", traj_end, load_step),
        ("负载突变.3s", load_step, load_step + int(0.3 / dt)),
        ("负载后稳态", load_step + int(0.3 / dt), imp_step),
        ("冲击瞬态.3s", imp_step, imp_step + int(0.3 / dt)),
        ("冲击后恢复", imp_step + int(0.3 / dt), res["n_steps"]),
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

    print("\n========== 每关节精度（真实误差, 单位 rad）==========")
    print(f"{'j':>3} {'e轨迹':>9} {'e静止':>9} {'e负载':>9} {'e冲击':>9} "
          f"{'max|tau|':>9} {'max|ff|':>9} {'max|fb|':>9} {'max|扰动|':>9} "
          f"{'饱和%':>7} {'恢复s':>7}")
    for j, p in enumerate(summary["per_joint"]):
        print(f"{j+1:>3} {p['e_traj']:>9.5f} {p['e_ss']:>9.5f} "
              f"{p['e_load']:>9.5f} {p['e_impact']:>9.5f} "
              f"{p['tau_max']:>9.3f} {p['ff_max']:>9.3f} {p['fb_max']:>9.3f} "
              f"{p['dist_max']:>9.3f} {p['sat_pct']:>7.2f} {p['settle']:>7.3f}")
    print(f"(e轨迹 = t<{s.T_TRAJ:g}s 运动段; e静止 = 轨迹后到 t={s.PAYLOAD_T:g}s, "
          f"主要由摩擦造成的静差)")
    print(f"(e负载 = 末端加 {s.PAYLOAD_MASS:g} kg 后 1 s 起的新稳态; "
          f"e冲击 = t>={s.IMPACT_T:g}s 力矩脉冲段峰值)")
    print("(max|扰动| = 作用在被控对象上的摩擦+冲击力矩; 饱和% = |dtau| 触及该关节 "
          f"DTAU_MAX={np.array2string(res['dtau_max'], precision=3, separator=', ')} Nm 的占比)")
    print(f"(恢复s = 冲击后 |e| 回到 {settle_tol} rad 内所需时间, nan=未恢复)")
    print(f"总力矩被 effort={np.array2string(res['effort'], precision=1, separator=', ')}"
          f" 截断的关节-周期数: {res['n_tau_saturated']}"
          f" / {res['n_steps'] * n_joints}"
          f"  (QP 里已有约束, 这里是错峰 hold 导致的兜底截断)")
    print(f"环路延迟: 执行 {s.CTRL_DELAY_TICKS} tick + 反馈 "
          f"{s.FEEDBACK_DELAY_TICKS} tick = "
          f"{(s.CTRL_DELAY_TICKS + s.FEEDBACK_DELAY_TICKS) * res['dt'] * 1e3:.1f} ms"
          f"  (MPC_DT = {res['mpc_dt'] * 1e3:.1f} ms)")

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

    is_cs = s.SOLVER == "cs_admm"
    if is_cs:
        print("\n========== 求解器参数（消元 + 定迭代 ADMM）==========")
        names = ("MPC_BLOCKS", "ADMM_ITERS", "ADMM_RHO_U", "ADMM_RHO_S", "ADMM_ALPHA",
                 "SOFT_POS_WEIGHT", "SOFT_VEL_WEIGHT", "E_CLAMP_FACTOR")
    else:
        print("\n========== OSQP 参数 ==========")
        names = ("OSQP_RHO", "OSQP_EPS_ABS", "OSQP_EPS_REL", "OSQP_MAX_ITER",
                 "OSQP_CHECK_TERMINATION", "OSQP_WARM_START",
                 "E_CLAMP_FACTOR", "USE_UNCONVERGED_ITERATE")
    for name in names:
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
    if is_cs:
        print(f"(定迭代 {s.ADMM_ITERS} 次，每拍解全部关节，耗时与工况无关)")
    else:
        print(f"(max_iter={s.OSQP_MAX_ITER}; H7@480MHz 单次迭代约 1~3 us, "
              f"1 ms 预算内目标 iter_max <= 100)")

    print("\n========== 每关节求解器性能 ==========")
    print(f"{'j':>3} {'次数':>7} {'iter_mean':>10} {'iter_max':>9} {'钳位%':>8}")
    for j in range(n_joints):
        m = res["solve_joint"] == j
        print(f"{j+1:>3} {int(m.sum()):>7} {res['solve_iter'][m].mean():>10.1f} "
              f"{int(res['solve_iter'][m].max()):>9} "
              f"{100.0*res['solve_clamped'][m].mean():>8.2f}")
