"""从 URDF 推 Craig MDH 表和 MDH 连杆系下的动力学参数, 并和 ins_arm.c 里的表逐项比对.

约定 (和负责人确认过):
  - 固件的关节角 q 就是 URDF 的关节角: MDH 的 z_i 沿 URDF 的 axis 方向, q = 0 时
    MDH 每个连杆系和 URDF 零位姿态下的转轴重合, 零位差全部放进 offset.
    前馈用的 rnea() 是从同一个 URDF 生成的, 吃的就是这个 q.
  - 第 0 系 = URDF 的 base 系 (重力沿 -z0).
  - 第 6 系原点和 URDF link6 原点重合; 姿态允许差一个固定旋转 (会打印出来).

检查三件事:
  1. 推出来的 MDH 链和 URDF 链在随机构型下每个关节的原点、转轴都吻合.
  2. ins_arm.c 里的 mdh / mass / centroid / inertia 四张表和推出来的一致.
  3. 打印推出来的表, 换 URDF 以后可以直接抄进 ins_arm.c.

纯标准库实现, 不依赖 numpy.
用法 (仓库根目录): python tests/arm_model/verify_mdh.py [URDF 路径]
"""
import os
import sys
import math
import random
import re

URDF = sys.argv[1] if len(sys.argv) > 1 else "D:/rm/urdf_urdf/urdf.urdf"
INS_ARM_C = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..",
                         "application", "gimbal", "Instance", "Src", "ins_arm.c")


# ---------------------------------------------------------------- 向量小工具
def add(a, b): return [a[i] + b[i] for i in range(3)]
def sub(a, b): return [a[i] - b[i] for i in range(3)]
def mul(a, s): return [a[i] * s for i in range(3)]
def dot(a, b): return sum(a[i] * b[i] for i in range(3))
def cross(a, b): return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
def norm(a): return math.sqrt(dot(a, a))
def unit(a): n = norm(a); return [x / n for x in a]
def matvec(M, v): return [dot(M[i], v) for i in range(3)]
def matT(M): return [[M[j][i] for j in range(3)] for i in range(3)]
def matmul(A, B): return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def rot_axis(k, ang):
    kx, ky, kz = k
    c, s, v = math.cos(ang), math.sin(ang), 1 - math.cos(ang)
    return [[kx * kx * v + c, kx * ky * v - kz * s, kx * kz * v + ky * s],
            [ky * kx * v + kz * s, ky * ky * v + c, ky * kz * v - kx * s],
            [kz * kx * v - ky * s, kz * ky * v + kx * s, kz * kz * v + c]]


def rpy_to_R(r, p, y):
    return matmul(rot_axis([0, 0, 1], y), matmul(rot_axis([0, 1, 0], p), rot_axis([1, 0, 0], r)))


# ---------------------------------------------------------------- 读 URDF
def parse_urdf(path):
    s = open(path, encoding="utf-8").read()
    links = {}
    for m in re.finditer(r'<link name="([^"]+)">(.*?)</link>', s, re.S):
        body = m.group(2)
        im = re.search(r"<inertial>(.*?)</inertial>", body, re.S)
        if not im:
            continue
        ib = im.group(1)
        o = re.search(r'<origin xyz="([^"]+)" rpy="([^"]+)"', ib)
        mass = float(re.search(r'<mass value="([^"]+)"', ib).group(1))
        it = re.search(r"<inertia ([^/]+)/>", ib).group(1)
        d = dict((k, float(v)) for k, v in re.findall(r'(\w+)="([^"]+)"', it))
        I = [[d["ixx"], d["ixy"], d["ixz"]], [d["ixy"], d["iyy"], d["iyz"]], [d["ixz"], d["iyz"], d["izz"]]]
        links[m.group(1)] = dict(mass=mass, com=[float(x) for x in o.group(1).split()],
                                 com_rpy=[float(x) for x in o.group(2).split()], I=I)
    joints = []
    for m in re.finditer(r'<joint name="([^"]+)" type="revolute">(.*?)</joint>', s, re.S):
        b = m.group(2)
        o = re.search(r'<origin xyz="([^"]+)" rpy="([^"]+)"', b)
        joints.append(dict(name=m.group(1),
                           parent=re.search(r'<parent link="([^"]+)"', b).group(1),
                           child=re.search(r'<child link="([^"]+)"', b).group(1),
                           xyz=[float(x) for x in o.group(1).split()],
                           rpy=[float(x) for x in o.group(2).split()],
                           axis=[float(x) for x in re.search(r'<axis xyz="([^"]+)"', b).group(1).split()]))
    return links, joints


def urdf_fk(joints, q):
    """返回每个关节（= child link）坐标系在 base 下的 (R, p)。"""
    R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    p = [0.0, 0.0, 0.0]
    out = []
    for j, qi in zip(joints, q):
        p = add(p, matvec(R, j["xyz"]))
        R = matmul(R, rpy_to_R(*j["rpy"]))
        R = matmul(R, rot_axis(unit(j["axis"]), qi))
        out.append(([row[:] for row in R], p[:]))
    return out


# ---------------------------------------------------------------- MDH 推导
def derive_mdh(joints):
    """Craig MDH。返回 [(offset, d, a, alpha)] 和 q=0 时各 MDH 系 (R, p)。"""
    fk0 = urdf_fk(joints, [0.0] * len(joints))
    n = len(joints)
    z = [unit(matvec(R, unit(j["axis"]))) for (R, p), j in zip(fk0, joints)]
    P = [p for (R, p) in fk0]  # 每根转轴上的一点（关节原点）

    # 第 i 系原点 O_i 在轴 i 上，x_i 沿轴 i 到轴 i+1 的公垂线
    O = [None] * n
    X = [None] * n
    for i in range(n - 1):
        c = cross(z[i], z[i + 1])
        if norm(c) > 1e-9:  # 不平行：公垂线
            xi = unit(c)
            # 求轴 i 上 t、轴 i+1 上 s，使连线沿 xi
            w = sub(P[i + 1], P[i])
            # P[i] + t z_i + a x_i = P[i+1] + s z_{i+1}
            # 两边点乘 z_i、z_{i+1}，解 2x2
            a11, a12 = dot(z[i], z[i]), -dot(z[i + 1], z[i])
            a21, a22 = dot(z[i], z[i + 1]), -dot(z[i + 1], z[i + 1])
            b1, b2 = dot(w, z[i]), dot(w, z[i + 1])
            det = a11 * a22 - a12 * a21
            t = (b1 * a22 - a12 * b2) / det
            O[i] = add(P[i], mul(z[i], t))
            foot = add(P[i + 1], mul(z[i + 1], (a11 * b2 - a21 * b1) / det))
            # x 指向轴 i+1（a >= 0）；两轴相交时取叉积方向
            dv = sub(foot, O[i])
            if norm(dv) > 1e-9:
                xi = unit(dv)
            X[i] = xi
        else:  # 平行：公垂线不唯一，取过关节 i 原点的那一条
            w = sub(P[i + 1], P[i])
            perp = sub(w, mul(z[i], dot(w, z[i])))
            O[i] = P[i][:]
            if norm(perp) > 1e-9:
                X[i] = unit(perp)
            else:  # 同轴：x 任取，沿上一系的 x（或 base x）
                X[i] = None
    # 最后一系：原点 = link6 原点，x 沿上一系 x 在垂直 z6 平面上的投影
    O[n - 1] = P[n - 1][:]
    X[n - 1] = None
    for i in range(n):
        if X[i] is None:
            ref = X[i - 1] if i > 0 and X[i - 1] is not None else [1.0, 0.0, 0.0]
            v = sub(ref, mul(z[i], dot(ref, z[i])))
            if norm(v) < 1e-9:
                v = sub([0, 1, 0], mul(z[i], dot([0, 1, 0], z[i])))
            X[i] = unit(v)

    # 第 0 系 = base
    Rb = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    frames = [(Rb, [0.0, 0.0, 0.0])]
    for i in range(n):
        Ri = matT([X[i], cross(z[i], X[i]), z[i]])  # 列向量 x, y, z
        frames.append((Ri, O[i]))

    mdh = []
    for i in range(n):
        Rp, Op = frames[i]
        Rc, Oc = frames[i + 1]
        xp = [Rp[0][0], Rp[1][0], Rp[2][0]]
        zp = [Rp[0][2], Rp[1][2], Rp[2][2]]
        yp = [Rp[0][1], Rp[1][1], Rp[2][1]]
        zc = [Rc[0][2], Rc[1][2], Rc[2][2]]
        xc = [Rc[0][0], Rc[1][0], Rc[2][0]]
        # alpha_{i-1}: 绕 x_{i-1} 从 z_{i-1} 转到 z_i
        alpha = math.atan2(dot(cross(zp, zc), xp), dot(zp, zc))
        # a_{i-1}: 沿 x_{i-1} 的距离；d_i: 沿 z_i 的距离
        w = sub(Oc, Op)
        a = dot(w, xp)
        # 检查：O_i - O_{i-1} = a x_{i-1} + d z_i
        d = dot(sub(w, mul(xp, a)), zc)
        res = sub(sub(w, mul(xp, a)), mul(zc, d))
        assert norm(res) < 1e-9, ("frame placement inconsistent", i, res)
        # theta offset: 绕 z_i 从 x_{i-1}(转 alpha 后) 到 x_i
        x_after = xp  # Rx(alpha) 不改变 x
        theta = math.atan2(dot(cross(x_after, xc), zc), dot(x_after, xc))
        mdh.append((theta, d, a, alpha))
    return mdh, frames


def mdh_fk(mdh, q):
    R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    p = [0.0, 0.0, 0.0]
    out = []
    for (off, d, a, al), qi in zip(mdh, q):
        th = qi + off
        ct, st, ca, sa = math.cos(th), math.sin(th), math.cos(al), math.sin(al)
        T = [[ct, -st, 0.0], [st * ca, ct * ca, -sa], [st * sa, ct * sa, ca]]
        t = [a, -sa * d, ca * d]
        p = add(p, matvec(R, t))
        R = matmul(R, T)
        out.append(([row[:] for row in R], p[:]))
    return out


# ---------------------------------------------------------------- 读 ins_arm.c
def parse_ins_arm(path):
    s = open(path, encoding="utf-8").read()

    def table(name):
        m = re.search(name + r"\[[^=]*=\s*\{(.*?)\n\s*\};", s, re.S)
        body = re.sub(r"/\*.*?\*/", "", m.group(1), flags=re.S)
        body = body.replace("EC_PI", repr(math.pi))
        nums = re.findall(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?(?:\s*/\s*\d+(?:\.\d*)?)?", body)
        out = []
        for t in nums:
            if "/" in t:
                a, b = t.split("/")
                out.append(float(a) / float(b))
            else:
                out.append(float(t))
        return out

    mdh = table("mdh")
    return dict(mdh=[mdh[4 * i:4 * i + 4] for i in range(6)], mass=table("mass"),
                centroid=table("centroid"), inertia=table("inertia"))


def main():
    links, joints = parse_urdf(URDF)
    mdh, frames = derive_mdh(joints)
    print("MDH (offset, d, a, alpha):")
    for row in mdh:
        print("  {%.9ff, %.9ff, %.9ff, %.9ff}," % row)

    # 校验：随机构型下每个关节原点、转轴都要吻合
    random.seed(0)
    worst_p = worst_z = 0.0
    tool = None
    worst_tool = 0.0
    for _ in range(2000):
        q = [random.uniform(-3.14, 3.14) for _ in joints]
        fu = urdf_fk(joints, q)
        fm = mdh_fk(mdh, q)
        for i, j in enumerate(joints):
            Ru, pu = fu[i]
            Rm, pm = fm[i]
            zu = matvec(Ru, unit(j["axis"]))
            zm = [Rm[0][2], Rm[1][2], Rm[2][2]]
            worst_z = max(worst_z, norm(sub(zu, zm)))
            # 关节原点在轴上：MDH 原点和 URDF 原点之差必须平行于轴
            dpv = sub(pm, pu)
            worst_p = max(worst_p, norm(sub(dpv, mul(zu, dot(dpv, zu)))))
        # 第 6 系原点完全重合；姿态差一个常数
        Ru, pu = fu[-1]
        Rm, pm = fm[-1]
        worst_p = max(worst_p, norm(sub(pm, pu)))
        Rt = matmul(matT(Rm), Ru)
        if tool is None:
            tool = Rt
        worst_tool = max(worst_tool, max(abs(Rt[a][b] - tool[a][b]) for a in range(3) for b in range(3)))
    print("max 原点偏差 %.3e m, 转轴偏差 %.3e, 工具系旋转不恒定 %.3e" % (worst_p, worst_z, worst_tool))
    print("R_mdh6 -> R_urdf_link6 =", [[round(x, 9) for x in r] for r in tool])

    # 动力学参数换到 MDH 连杆系（q = 0 时）
    fu0 = urdf_fk(joints, [0.0] * len(joints))
    print("\nmass / centroid (MDH 系) / inertia (绕质心, MDH 系):")
    derived = []
    for i, j in enumerate(joints):
        L = links[j["child"]]
        Ru, pu = fu0[i]
        Rm, pm = frames[i + 1]
        Rc = matmul(Ru, rpy_to_R(*L["com_rpy"]))
        com_w = add(pu, matvec(Ru, L["com"]))
        com_m = matvec(matT(Rm), sub(com_w, pm))
        I_w = matmul(Rc, matmul(L["I"], matT(Rc)))
        I_m = matmul(matT(Rm), matmul(I_w, Rm))
        print("  %d mass %.8f" % (i, L["mass"]))
        print("    com  {%.9ff, %.9ff, %.9ff}" % tuple(com_m))
        print("    I    " + ", ".join("%.9ef" % I_m[a][b] for a in range(3) for b in range(3)))
        derived.append((L["mass"], com_m, [I_m[a][b] for a in range(3) for b in range(3)]))

    # ins_arm.c 里的表必须和推出来的一致 (表里是 float, 容差按 float 精度给)
    fw = parse_ins_arm(INS_ARM_C)
    worst = dict(mdh=0.0, mass=0.0, centroid=0.0, inertia=0.0)
    for i in range(len(joints)):
        for k in range(4):
            dv = abs(fw["mdh"][i][k] - mdh[i][k])
            if k in (0, 3):  # 角度按 2pi 取模
                dv = min(dv, abs(dv - 2 * math.pi))
            worst["mdh"] = max(worst["mdh"], dv)
        m, c, I = derived[i]
        worst["mass"] = max(worst["mass"], abs(fw["mass"][i] - m))
        worst["centroid"] = max(worst["centroid"], max(abs(fw["centroid"][3 * i + k] - c[k]) for k in range(3)))
        worst["inertia"] = max(worst["inertia"], max(abs(fw["inertia"][9 * i + k] - I[k]) for k in range(9)))
    print("\nins_arm.c 和 URDF 推导结果的最大偏差: " +
          ", ".join("%s %.2e" % (k, v) for k, v in worst.items()))
    ok = (worst_p < 1e-9 and worst_z < 1e-9 and worst_tool < 1e-9 and
          worst["mdh"] < 1e-6 and worst["mass"] < 1e-7 and
          worst["centroid"] < 1e-7 and worst["inertia"] < 1e-9)
    print("RESULT: %s" % ("PASS" if ok else "FAIL"))
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
