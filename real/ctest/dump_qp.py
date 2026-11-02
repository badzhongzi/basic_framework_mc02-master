"""导出闭环仿真里的单次求解输入输出，供 C 侧 mpc_cs_* 做 float32 数值回归。

为什么要这个：仿真跑 float64，上板跑 float32，两边的 Δτ 必然有差异。
上板前必须知道这个差异有多大，否则所有仿真结论都悬着。

每条记录带求解**之前**的热启动状态 (u, zu, yu, zs, ys)，C 侧有两种回放方式：
  逐条  每条都从 Python 的热启动状态开始，只验这一次求解（最严格）
  连续  每段第一条拷状态，之后用 C 自己的状态往下走，验误差会不会累积
记录的是钳位前的 anchor_raw，C 的 mpc_cs_step() 内部自己钳位，顺带验证钳位上限。
同时导出 Python 算的控制量边界，单独比对 C 的边界构造。

场景（覆盖约束不起作用和起作用两种情况）：
  标准     轨迹运动段、负载突变、冲击
  速度限位 J0 1 s 走 2.5 rad，参考峰值 4.7 rad/s > 限位 3 rad/s（P5 速度约束起作用）
  位置限位 J1 目标 3.30，越过约束线 3.12（P5 位置约束起作用）
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import mpc_config as cfg                           # noqa: E402
import cmpc                                        # noqa: E402
import sim_core                                    # noqa: E402

OUT = Path(__file__).resolve().parent / "qp_cases.txt"


def _scenarios():
    dt = cfg.BASE_DT
    st = lambda t: int(round(t / dt))
    q_vel = list(cfg.Q_GOAL); q_vel[0] = 2.5
    q_pos = list(cfg.Q_GOAL); q_pos[1] = 3.30
    return [
        ("标准", {"T_SIM": cfg.IMPACT_T + 0.3},
         [(st(2.0), st(2.4)), (st(cfg.PAYLOAD_T) - 10, st(cfg.PAYLOAD_T) + 100),
          (st(cfg.IMPACT_T) - 10, st(cfg.IMPACT_T) + 100)]),
        ("速度限位", {"Q_GOAL": q_vel, "T_TRAJ": 1.0, "T_SIM": 1.5},
         [(st(0.2), st(0.8))]),
        ("位置限位", {"Q_GOAL": q_pos, "T_SIM": cfg.T_TRAJ + 0.5},
         [(st(cfg.T_TRAJ) - 300, st(cfg.T_TRAJ) + 100)]),
    ]


def main():
    lines = []
    seg = 0
    summary = []
    for name, ov, windows in _scenarios():
        steps = set()
        for a, b in windows:
            steps.update(range(a, b))
        res = sim_core.run(overrides=ov, qp_record_steps=steps)
        s = res["settings"]
        if s.SOLVER != "cs_admm":
            raise SystemExit("C 侧只实现了 SOLVER='cs_admm'")
        d = res["cs"]
        recs = res["qp_records"]
        n_bound = 0
        for a, b in windows:
            for r in recs:
                if not (a <= r["step"] < b):
                    continue
                j = r["joint"]
                lo_u, hi_u = cmpc.control_bounds_blocks(
                    d, res["dtau_max"][j], res["effort"][j],
                    r["tau_ff"], r["tau_ff_end"], r["rho"])
                if min(abs(r["u_sol"] - lo_u[0]), abs(r["u_sol"] - hi_u[0])) < 1e-9:
                    n_bound += 1
                vals = [seg, j]
                vals += list(r["anchor_raw"]) + list(r["anchor"])
                vals += [r["w_hat"], r["tau_ff"], r["tau_ff_end"], r["rho"]]
                vals += list(r["qr_e"]) + list(r["dqr_e"])
                for w in r["warm"]:
                    vals += list(w)
                vals += list(lo_u) + list(hi_u)
                vals += [r["u_sol"]]
                lines.append(" ".join(f"{v:.17g}" if isinstance(v, float) else str(v)
                                      for v in (float(x) if not isinstance(x, int) else x
                                                for x in vals)))
            seg += 1
        summary.append((name, len(recs), n_bound,
                        np.abs(res["dq"][:, 0]).max(), res["q"][:, 1].max()))

    with open(OUT, "w", encoding="ascii", newline="\n") as f:
        f.write(f"{res['n_joints']} {d['NU']} {d['NB']} {len(lines)}\n")
        f.write("\n".join(lines) + "\n")
    print(f"写入 {OUT}: {len(lines)} 次求解, {seg} 段")
    for name, n, nb, dq0, q1 in summary:
        print(f"  {name:6s} {n:5d} 次求解, 首个控制量顶在箱约束上 {nb} 次, "
              f"J0 max|dq| {dq0:.3f}, J1 max q {q1:.4f}")


if __name__ == "__main__":
    main()
