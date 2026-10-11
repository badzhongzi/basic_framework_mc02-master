# sweep_rt.py
# 实时性预算扫描：找一组能把 iter_max 压到 100 以内、又不损失精度的 OSQP 配置。
#
# 目标板 STM32H7 @480MHz。JOINTS_PER_TICK=1、BASE_DT=1ms，每个 1 ms tick 内要跑完
# 一次 6 自由度 RNEA 加一次完整 QP，所以预算是 1 ms。nz=nc=30 的 float32 嵌入式
# OSQP 单次迭代约 1~3 us，留出 RNEA 和中断余量后 iter_max 目标 <= 100。
#
# 四个杠杆逐个扫，再跑一组组合候选。判据：
#   iter_max <= 100、e轨迹不退化（<= 1e-4）、恢复s 相对基线退化 <= 20%、未恢复数 = 0。
import warnings
warnings.filterwarnings("ignore", category=RuntimeWarning)

import time

import mpc_config as cfg
import sim_core

# 扫描用的缩短时长：轨迹 5.5 s + 扰动在 8 s + 6 s 恢复窗口，足够覆盖全部阶段。
# 实测最长恢复 1.76 s，6 s 窗口有余量。
T_SIM_SWEEP = 14.0

TARGET_ITER_MAX = 100
TARGET_E_TRAJ = 1.0e-4

CASES = []


def add(label, **overrides):
    overrides.setdefault("T_SIM", T_SIM_SWEEP)
    CASES.append((label, overrides))


# ---- 基线：当前 mpc_config 的值，但关掉钳位，还原扫描前的行为 ----
add("基线(无钳位)", E_CLAMP_FACTOR=None, USE_UNCONVERGED_ITERATE=False)

# ---- 杠杆 1：误差钳位系数 ----
for f in (10.0, 5.0, 3.0, 2.0, 1.5, 1.0):
    add(f"钳位 factor={f:g}", E_CLAMP_FACTOR=f)

# ---- 杠杆 2：收敛容差。Δτ 量级 12 Nm，1e-3 的精度对执行器无影响 ----
for e in (1.0e-4, 3.0e-4, 1.0e-3, 3.0e-3):
    add(f"eps={e:g}", E_CLAMP_FACTOR=2.0, OSQP_EPS_ABS=e, OSQP_EPS_REL=e)

# ---- 杠杆 3：终止检查间隔。默认 25 把 iter 量化成 25 的倍数 ----
for c in (25, 10, 5, 1):
    add(f"check_term={c}", E_CLAMP_FACTOR=2.0, OSQP_CHECK_TERMINATION=c)

# ---- 杠杆 4：max_iter 硬上限 + 用未收敛迭代点 ----
for m in (2000, 200, 100, 60, 40):
    add(f"max_iter={m}", E_CLAMP_FACTOR=2.0, OSQP_MAX_ITER=m,
        USE_UNCONVERGED_ITERATE=True)

# ---- 组合候选 ----
add("组合A 稳", E_CLAMP_FACTOR=2.0, OSQP_EPS_ABS=1e-3, OSQP_EPS_REL=1e-3,
    OSQP_CHECK_TERMINATION=5, OSQP_MAX_ITER=100, USE_UNCONVERGED_ITERATE=True)
add("组合B 紧", E_CLAMP_FACTOR=1.5, OSQP_EPS_ABS=1e-3, OSQP_EPS_REL=1e-3,
    OSQP_CHECK_TERMINATION=5, OSQP_MAX_ITER=60, USE_UNCONVERGED_ITERATE=True)
add("组合C 极紧", E_CLAMP_FACTOR=1.5, OSQP_EPS_ABS=3e-3, OSQP_EPS_REL=3e-3,
    OSQP_CHECK_TERMINATION=1, OSQP_MAX_ITER=40, USE_UNCONVERGED_ITERATE=True)


HEAD = (f"{'配置':<16} {'it_mean':>8} {'it_max':>7} {'it_p99':>7} {'未收敛':>7} "
        f"{'钳位%':>7} {'e轨迹':>10} {'e稳态':>10} {'饱和%':>7} "
        f"{'恢复s':>8} {'未恢复':>7} {'判定':>6}")

print(f"T_SIM={T_SIM_SWEEP} s, 共 {len(CASES)} 组配置")
print(f"目标: iter_max <= {TARGET_ITER_MAX}, e轨迹 <= {TARGET_E_TRAJ:g}, 未恢复=0\n")
print(HEAD)
print("-" * len(HEAD))

rows = []
baseline_settle = None
for label, overrides in CASES:
    t0 = time.perf_counter()
    res = sim_core.run(overrides)
    sm = sim_core.summarize(res)
    wall = time.perf_counter() - t0

    ok = (sm["iter_max"] <= TARGET_ITER_MAX
          and sm["e_traj_max"] <= TARGET_E_TRAJ
          and sm["n_not_settled"] == 0
          and sm["n_other"] == 0)
    if baseline_settle is None:
        baseline_settle = sm["settle_max"]
    elif ok and sm["settle_max"] > baseline_settle * 1.2:
        ok = False          # 恢复时间相对基线退化超过 20%

    rows.append((label, sm, ok, wall))
    print(f"{label:<16} {sm['iter_mean']:>8.1f} {sm['iter_max']:>7} "
          f"{sm['iter_p99']:>7.0f} {sm['n_unconverged']:>7} "
          f"{sm['clamp_pct']:>7.2f} {sm['e_traj_max']:>10.2e} "
          f"{sm['e_ss_max']:>10.2e} {sm['sat_pct_max']:>7.2f} "
          f"{sm['settle_max']:>8.3f} {sm['n_not_settled']:>7} "
          f"{'OK' if ok else '-':>6}")

print("-" * len(HEAD))
print(f"(判定 OK = iter_max<={TARGET_ITER_MAX} 且 e轨迹<={TARGET_E_TRAJ:g} 且 "
      f"全部恢复 且 恢复s 不超基线 {baseline_settle:.3f}s 的 1.2 倍)")
print(f"(恢复s 取六关节最大值; 未恢复 = 仿真结束仍超 "
      f"{sim_core.SETTLE_TOL} rad 的关节数)")

passed = [r for r in rows if r[2]]
if passed:
    best = min(passed, key=lambda r: (r[1]["iter_max"], r[1]["settle_max"]))
    print(f"\n最优通过配置: {best[0]}  "
          f"iter_max={best[1]['iter_max']}, 恢复s={best[1]['settle_max']:.3f}")
else:
    print("\n没有配置同时满足全部判据，需要换杠杆（减小 N 或重新调 rho）。")

total = sum(r[3] for r in rows)
print(f"总耗时 {total:.1f} s")
