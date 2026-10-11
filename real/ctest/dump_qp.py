"""导出一段闭环仿真里的 QP 输入输出，供 C 侧做 float32 数值回归。

为什么要这个：仿真跑 float64 的 pip osqp，上板跑 float32 的嵌入式 osqp，
两边的 Δτ 必然有差异。上板前必须知道这个差异有多大，否则所有仿真结论都悬着。

窗口取 [T0_STEP, T1_STEP)，含扰动瞬态。起点选在稳态（误差 ~1e-8、Δτ ~0），
这样 C 侧冷启动的热启动状态和 Python 在该时刻的状态基本一致，
不会把启动瞬态误当成精度问题。

记录的是钳位前的 anchor_raw，因为 C 的 mpc_build_vectors() 内部自己钳位——
这样顺带验证两边的钳位上限一致。
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import sim_core                                    # noqa: E402

T0_STEP = 7000
T1_STEP = 8150
OUT = Path(__file__).resolve().parent / "qp_cases.txt"


def main():
    res = sim_core.run(qp_record_steps=range(T0_STEP, T1_STEP))
    recs = res["qp_records"]
    N = res["settings"].N
    nj = res["n_joints"]
    if not recs:
        raise SystemExit("没有记录到任何 QP，检查 T0_STEP/T1_STEP 是否超出 T_SIM")

    n_clamped = sum(1 for r in recs if not np.array_equal(r["anchor"], r["anchor_raw"]))
    with open(OUT, "w", encoding="ascii", newline="\n") as f:
        f.write(f"{nj} {N} {len(recs)}\n")
        for r in recs:
            vals = [r["joint"]]
            vals += list(r["anchor_raw"])
            vals += list(r["anchor"])              # C 侧用来校验钳位上限
            vals += list(np.asarray(r["delta_seq"]).ravel())
            vals += [r["dtau"]]
            f.write(" ".join(f"{v:.17g}" if isinstance(v, float) else str(v)
                             for v in vals) + "\n")

    dt = np.abs([r["dtau"] for r in recs])
    print(f"写入 {OUT}")
    print(f"  记录 {len(recs)} 次求解, 步 [{T0_STEP}, {T1_STEP}), N={N}, 关节数={nj}")
    print(f"  触发钳位 {n_clamped} 次 ({100.0*n_clamped/len(recs):.1f}%)")
    print(f"  |dtau| 范围 [{dt.min():.6g}, {dt.max():.6g}], "
          f"顶满 DTAU_MAX 的 {int(np.sum(dt >= res['settings'].DTAU_MAX*0.999))} 次")


if __name__ == "__main__":
    main()
