"""校验从 openarm.urdf 推导出的 MDH 参数表.

对比两条独立计算的正运动学链:
  1. URDF 原生链: 逐关节 Trans(origin_xyz) * Rot(axis, q)
  2. MDH 链:      逐连杆 Rx(alpha_{i-1}) Tx(a_{i-1}) Rz(q_i + offset_i) Tz(d_i)
                  (与 Engineer_Components_master/src/ec_transform.c:ec_mdh_to_transform 同式)

末端位置必须逐点吻合; 姿态允许相差一个常量右乘 (末端工具系定义自由度),
因此额外检查该常量在所有随机构型下保持一致.

同时输出各连杆质心在 MDH 连杆系下的坐标, 供 ins_arm_init 填写
ec_link_set_dynamic 的 centroid 参数.

纯标准库实现, 不依赖 numpy.
用法: python tests/arm_model/verify_mdh.py
"""

import math
import random

PI = math.pi

# URDF: 6 个 revolute 关节, 每个 origin xyz=(0,0,0.1) rpy=0, 轴序 z,y,y,z,y,z
URDF_ORIGIN = (0.0, 0.0, 0.1)
URDF_AXES = [
    (0.0, 0.0, 1.0),
    (0.0, 1.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
]
# URDF 各连杆质心, 表达在自身 link 系
URDF_COM_LOCAL = (0.0, 0.0, 0.05)

# 待校验的 MDH 表: (offset, d, a, alpha), 其中 alpha/a 实为 alpha_{i-1}/a_{i-1}
MDH = [
    (0.0, 0.2, 0.0, 0.0),
    (-PI / 2, 0.0, 0.0, -PI / 2),
    (+PI / 2, 0.0, 0.1, 0.0),
    (0.0, 0.2, 0.0, +PI / 2),
    (0.0, 0.0, 0.0, -PI / 2),
    (0.0, 0.1, 0.0, +PI / 2),
]


def mat_mul(a, b):
    """4x4 齐次矩阵相乘, 行主序的 4 行 4 列嵌套列表."""
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]


def identity():
    return [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]


def urdf_joint_transform(axis, angle, origin):
    """Trans(origin) * Rot(axis, angle), 轴为单位向量时的 Rodrigues 展开."""
    kx, ky, kz = axis
    c, s, v = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    r = [
        [kx * kx * v + c,      kx * ky * v - kz * s, kx * kz * v + ky * s],
        [ky * kx * v + kz * s, ky * ky * v + c,      ky * kz * v - kx * s],
        [kz * kx * v - ky * s, kz * ky * v + kx * s, kz * kz * v + c],
    ]
    return [r[0] + [origin[0]], r[1] + [origin[1]], r[2] + [origin[2]],
            [0.0, 0.0, 0.0, 1.0]]


def mdh_transform(theta, d, a, alpha):
    """与 ec_mdh_to_transform 完全一致的矩阵 (Craig / modified DH)."""
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return [
        [ct,      -st,     0.0, a],
        [st * ca, ct * ca, -sa, -sa * d],
        [st * sa, ct * sa, ca,  ca * d],
        [0.0,     0.0,     0.0, 1.0],
    ]


def urdf_chain(q):
    """URDF 原生正解, 返回 base 与 6 个 link 系的位姿, 共 7 项."""
    out = [identity()]
    t = identity()
    for i in range(6):
        t = mat_mul(t, urdf_joint_transform(URDF_AXES[i], q[i], URDF_ORIGIN))
        out.append(t)
    return out


def mdh_chain(q):
    """MDH 正解, 返回 base 与 6 个连杆系的位姿, 共 7 项."""
    out = [identity()]
    t = identity()
    for i in range(6):
        offset, d, a, alpha = MDH[i]
        t = mat_mul(t, mdh_transform(q[i] + offset, d, a, alpha))
        out.append(t)
    return out


def rot_of(t):
    return [row[:3] for row in t[:3]]


def pos_of(t):
    return [t[i][3] for i in range(3)]


def rot_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def rot_t_vec(r, v):
    """R^T * v."""
    return [sum(r[k][i] * v[k] for k in range(3)) for i in range(3)]


def rot_vec(r, v):
    return [sum(r[i][k] * v[k] for k in range(3)) for i in range(3)]


def transpose3(r):
    return [[r[j][i] for j in range(3)] for i in range(3)]


def main():
    rng = random.Random(20260921)
    tests = [[0.0] * 6]
    tests += [[rng.uniform(-PI, PI) for _ in range(6)] for _ in range(500)]

    # 末端姿态的常量偏差由 q=0 构型定出, 其余构型必须保持一致
    r_tool = rot_mul(transpose3(rot_of(mdh_chain(tests[0])[-1])),
                     rot_of(urdf_chain(tests[0])[-1]))

    max_pos_err = 0.0
    max_rot_err = 0.0
    for q in tests:
        t_u = urdf_chain(q)[-1]
        t_m = mdh_chain(q)[-1]
        pu, pm = pos_of(t_u), pos_of(t_m)
        max_pos_err = max(max_pos_err, max(abs(pu[i] - pm[i]) for i in range(3)))
        pred = rot_mul(rot_of(t_m), r_tool)
        ru = rot_of(t_u)
        max_rot_err = max(max_rot_err,
                          max(abs(pred[i][j] - ru[i][j])
                              for i in range(3) for j in range(3)))

    print("samples              : %d" % len(tests))
    print("max |dp| (m)         : %.3e" % max_pos_err)
    print("max |dR| (tool-fixed): %.3e" % max_rot_err)
    print("tool rotation (MDH frame6 -> URDF link6):")
    for row in r_tool:
        print("   [%s]" % ", ".join("%+.0f" % x for x in row))

    print("")
    print("centroid in MDH link frame (for ec_link_set_dynamic):")
    urdf0 = urdf_chain([0.0] * 6)
    mdh0 = mdh_chain([0.0] * 6)
    for i in range(1, 7):
        com_world = [a + b for a, b in
                     zip(rot_vec(rot_of(urdf0[i]), list(URDF_COM_LOCAL)),
                         pos_of(urdf0[i]))]
        delta = [com_world[k] - pos_of(mdh0[i])[k] for k in range(3)]
        com_mdh = rot_t_vec(rot_of(mdh0[i]), delta)
        print("   link%d: {%+.4ff, %+.4ff, %+.4ff}"
              % (i, com_mdh[0], com_mdh[1], com_mdh[2]))

    ok = max_pos_err < 1e-12 and max_rot_err < 1e-12
    print("")
    print("RESULT: %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
