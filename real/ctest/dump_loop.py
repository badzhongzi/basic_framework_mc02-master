"""导出闭环仿真的逐步输入和输出，供 C 侧做**整个 tick 循环**的回归。

和 dump_qp.py 的区别：dump_qp 只验"单次求解一致"，这里验的是整条链：
RNEA 前馈、摩擦补偿、时域末尾前馈预测（两次参考 RNEA）、惯量比（P3）、
观测器 + 延迟补偿（P1/P2）、每拍全部关节求解（P4）、约束（P5）、
定迭代 ADMM（P7）、总力矩约束和装配饱和。

被控对象不在测试范围内：C 侧没有正向动力学（只有 RNEA/CRBA），
所以这里回放仿真记录的测量序列，只比对控制器的输出。

分三段：轨迹运动段（参考预览在变）、负载突变、冲击。每段起点从 Python 拷
观测器状态和求解器热启动状态，所以从第 0 步起就能逐步比对，不需要预热。
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import mpc_config as cfg                            # noqa: E402
import sim_core                                     # noqa: E402

OUT = Path(__file__).resolve().parent / "loop_cases.txt"
# 每步每关节一个数的块，顺序和 test_loop.c 一致
BLOCKS = ("q_meas", "dq_meas", "qr_m", "dqr_m", "qr_a", "dqr_a", "ddqr_a",
          "qr_end", "dqr_end", "ddqr_end", "tau_ff", "rho", "w_hat", "dtau", "tau")


def main():
    dt = cfg.BASE_DT
    st = lambda t: int(round(t / dt))
    segs = [(0, st(cfg.T_TRAJ + 0.5)),
            (st(cfg.PAYLOAD_T - 0.1), st(cfg.PAYLOAD_T + 0.9)),
            (st(cfg.IMPACT_T - 0.1), st(cfg.IMPACT_T + 0.9))]
    res = sim_core.run(overrides={"T_SIM": cfg.IMPACT_T + 1.0},
                       qp_record_steps=[a for a, _ in segs])
    s = res["settings"]
    if s.ESTIMATOR != "kalman" or s.SOLVER != "cs_admm":
        raise SystemExit("C 侧 tick 循环只实现了 ESTIMATOR='kalman'、SOLVER='cs_admm'")
    nj = res["n_joints"]
    d = res["cs"]
    D = int(res["est_D"])
    warm = {(r["step"], r["joint"]): r["warm"] for r in res["qp_records"]}
    fmt = lambda vals: " ".join(f"{float(v):.9g}" for v in vals)

    with open(OUT, "w", encoding="ascii", newline="\n") as f:
        f.write(f"{nj} {d['NU']} {d['NB']} {D} {len(segs)}\n")
        for a, b in segs:
            f.write(f"{a} {b - a}\n")
            for j in range(nj):        # 段起点更新之前的观测器状态
                f.write(fmt(list(res["est_z"][a][j]) + list(res["est_hist"][a][j])) + "\n")
            for j in range(nj):        # 段起点求解之前的热启动状态
                f.write(fmt(np.concatenate(warm[(a, j)])) + "\n")
            for i in range(a, b):
                vals = []
                for key in BLOCKS:
                    vals += list(res[key][i])
                vals += list(np.asarray(res["qr_e"][i]).ravel())   # [NB][NJ]
                vals += list(np.asarray(res["dqr_e"][i]).ravel())
                f.write(fmt(vals) + "\n")
    print(f"写入 {OUT}")
    for a, b in segs:
        print(f"  段 [{a}, {b}) t = [{a*dt:g}, {b*dt:g}] s")
    print(f"  每拍解全部 {nj} 个关节, 延迟补偿 D = {D}, 定迭代 {d['iters']} 次")


if __name__ == "__main__":
    main()
