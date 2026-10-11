# sim.py
# 单次闭环仿真：跑 sim_core.run()，打印统计，落盘 npz，最后画图。
# 闭环逻辑本身在 sim_core.py，sweep_rt.py 与本文件共用同一份内核和统计定义。
import warnings
warnings.filterwarnings("ignore", category=RuntimeWarning)

from pathlib import Path

import numpy as np

import mpc_config as cfg
import sim_core

# matplotlib 只在最后按需 import，这样 MAKE_PLOT=False 时完全不碰它。
SCRIPT_DIR = Path(__file__).resolve().parent

res = sim_core.run(progress=True, debug_steps=3)
summary = sim_core.summarize(res)

n_joints = res["n_joints"]
dtau_max = res["settings"].DTAU_MAX
sim_core.print_summary(res, summary)

# ================================================================
# 落盘原始数据。画图可能崩在 matplotlib 里，所以先存，
# 之后可以在任何装了可用 matplotlib 的环境里重画。
# ================================================================
npz_path = SCRIPT_DIR / "sim_result.npz"
np.savez_compressed(
    npz_path,
    t=res["t"], q=res["q"], dq=res["dq"], q_ref=res["q_ref"],
    dq_ref=res["dq_ref"], e=res["e"], e_dq=res["e_dq"],
    tau=res["tau"], tau_ff=res["tau_ff"], dtau=res["dtau"],
    solve_step=res["solve_step"], solve_joint=res["solve_joint"],
    solve_iter=res["solve_iter"], solve_status=res["solve_status"],
    solve_time_us=res["solve_time_us"], solve_clamped=res["solve_clamped"],
    solve_unconverged=res["solve_unconverged"],
    M_design=res["M_design"], K_pos=res["K_pos"], K_vel=res["K_vel"],
    base_dt=res["dt"], mpc_dt=res["mpc_dt"], N=cfg.N, n_joints=n_joints,
    q_pos=cfg.Q_POS, q_vel=cfg.Q_VEL, r_weight=cfg.R_WEIGHT,
    dtau_max=dtau_max, perturb_step=res["perturb_step"],
    perturb_dq=res["perturb_dq"],
)
print(f"\nSaved {npz_path}")

# ================================================================
# 画图（放最后）
# ================================================================
if not cfg.MAKE_PLOT:
    print("MAKE_PLOT=False，跳过画图。用 sim_result.npz 另行重画。")
    raise SystemExit(0)

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False

log_t = res["t"]
fig, axes = plt.subplots(n_joints, 3, figsize=(15, 14), sharex=True)
for i in range(n_joints):
    axes[i, 0].plot(log_t, res["q"][:, i], label=f'q{i+1}')
    axes[i, 0].plot(log_t, res["q_ref"][:, i], '--', alpha=0.5)
    axes[i, 0].set_ylabel(f'q{i+1} [rad]')
    axes[i, 0].grid(True)

    axes[i, 1].plot(log_t, res["e"][:, i], color='C3')
    axes[i, 1].set_ylabel(f'e{i+1} [rad]')
    axes[i, 1].grid(True)

    axes[i, 2].plot(log_t, res["tau"][:, i], label='tau', linewidth=1.3)
    axes[i, 2].plot(log_t, res["tau_ff"][:, i], '--', alpha=0.6, label='ff')
    axes[i, 2].plot(log_t, res["dtau"][:, i], ':', alpha=0.8, label='fb')
    axes[i, 2].axhline(dtau_max, color='k', lw=0.6, alpha=0.4)
    axes[i, 2].axhline(-dtau_max, color='k', lw=0.6, alpha=0.4)
    axes[i, 2].set_ylabel(f'tau{i+1} [Nm]')
    axes[i, 2].legend(loc='upper right', fontsize=8)
    axes[i, 2].grid(True)

for col in range(3):
    axes[-1, col].set_xlabel('t [s]')
plt.tight_layout()
result_path = SCRIPT_DIR / "sim_result.png"
plt.savefig(result_path, dpi=120)
print(f"Saved {result_path}")
