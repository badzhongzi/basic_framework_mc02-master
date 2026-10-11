"""导出 RNEA 用例，供 C 侧 float32 的 rnea_6axis.c 和 pinocchio 对数。

为什么必须做：tau = tau_ff + Δτ 里 tau_ff 是主项（实测 64~73 Nm，Δτ 只有 12 Nm）。
生成的 rnea_6axis.c 走的是 CasADi 符号展开 + float32，pinocchio 走 float64 递推。
两者算法上应当等价，只差舍入——但这件事从没验过。若前馈有结构性错误
（模型不对、关节顺序错、重力符号反），仿真的全部结论都不成立。

判据不取抽象的浮点容差，而是物理量：前馈误差对 MPC 来说和外部扰动无法区分，
所以它必须远小于修正权限 DTAU_MAX=12 Nm。真实工况下取 1% 即 0.12 Nm。

用例分三组，分开统计，这样"实际工况没问题但极端工况精度下降"这种情况看得出来：
  gravity  纯重力静止（保持位姿时最要紧的一项）
  sim      闭环仿真实际经历的 (q, dq, ddq_ref)，含扰动恢复段 dq 到 30 rad/s
  extreme  工作空间随机构型 + 人为拉大的 dq，用来压科氏/离心项
"""
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import sim_core                                    # noqa: E402

OUT = Path(__file__).resolve().parent / "rnea_cases.txt"
SIM_STRIDE = 13          # 从 30000 步里抽样，13 是质数，避免和 6 关节错峰周期共振
EXTREME_PER_SCALE = 200
DQ_SCALES = (0.0, 1.0, 5.0, 30.0, 130.0)   # 130 = 关掉钳位时实测的最大速度
SEED = 12345


def main():
    s = sim_core.Settings()
    model, data = sim_core.load_model(s.URDF_PATH)
    n = model.nv
    get_reference, _, _ = sim_core.make_reference(s, n)

    cases = []          # (group, q, dq, ddq)

    # ---- 组 1: 纯重力 ----
    rng = np.random.default_rng(SEED)
    cases.append(("gravity", np.zeros(n), np.zeros(n), np.zeros(n)))
    cases.append(("gravity", np.array(s.Q_GOAL, float), np.zeros(n), np.zeros(n)))
    _, lo, hi = sim_core.dyn.sample_poses(model, 0, SEED)
    for _ in range(100):
        cases.append(("gravity", lo + (hi - lo) * rng.random(n),
                      np.zeros(n), np.zeros(n)))

    # ---- 组 2: 闭环仿真实际经历的状态 ----
    res = sim_core.run()
    q_log, dq_log, t_log = res["q"], res["dq"], res["t"]
    for i in range(0, q_log.shape[0], SIM_STRIDE):
        _, _, ddq_ref = get_reference(t_log[i])
        cases.append(("sim", q_log[i].copy(), dq_log[i].copy(), ddq_ref.copy()))

    # ---- 组 3: 人为极端 ----
    for scale in DQ_SCALES:
        for _ in range(EXTREME_PER_SCALE):
            cases.append(("extreme",
                          lo + (hi - lo) * rng.random(n),
                          rng.uniform(-scale, scale, n),
                          rng.uniform(-20.0, 20.0, n)))

    groups = sorted({c[0] for c in cases})
    gidx = {g: i for i, g in enumerate(groups)}

    with open(OUT, "w", encoding="ascii", newline="\n") as f:
        f.write(f"{n} {len(cases)} {len(groups)}\n")
        f.write(" ".join(groups) + "\n")
        for grp, q, dq, ddq in cases:
            tau = np.array(pin.rnea(model, data, q, dq, ddq), dtype=float)
            if not np.all(np.isfinite(tau)):
                raise ValueError(f"pinocchio 返回非有限值, q={q} dq={dq} ddq={ddq}")
            vals = [str(gidx[grp])]
            vals += [f"{v:.17g}" for v in q]
            vals += [f"{v:.17g}" for v in dq]
            vals += [f"{v:.17g}" for v in ddq]
            vals += [f"{v:.17g}" for v in tau]
            f.write(" ".join(vals) + "\n")

    print(f"写入 {OUT}")
    print(f"  自由度 {n}, 用例 {len(cases)}")
    for g in groups:
        sub = [c for c in cases if c[0] == g]
        dqmax = max(float(np.abs(c[2]).max()) for c in sub)
        print(f"  {g:<8} {len(sub):>5} 例, max|dq| = {dqmax:.1f} rad/s")


if __name__ == "__main__":
    main()
