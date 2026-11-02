/* C 侧 tick 循环回归：回放仿真记录的测量序列，跑完整的控制链，比对每一步。
 *
 * 被测的是整条链（和 sim_core.run() 的循环体逐行对应）：
 *   RNEA 前馈 + 摩擦补偿
 *   -> 时域末尾的前馈预测：两次参考 RNEA，tau_ff_end = tau_ff + tref1 - tref0
 *   -> P3 惯量比 mpc_inertia_ratio()           (CRBA + 6x6 Cholesky)
 *   -> P1/P2 观测器 mpc_est_update / anchor    (延迟补偿前推)
 *   -> P4/P5/P7 每拍全部关节 mpc_cs_step()     (钳位、边界、定迭代 ADMM)
 *   -> Δτ = ρ·u，装配饱和，mpc_est_commit(饱和后反算的 u)
 *
 * 被控对象不在测试范围：C 侧只有逆动力学，没有正向动力学，所以回放测量值
 * 而不是真的闭环积分。被测的是控制器，不是对象。
 *
 * 编译见同目录 build.sh。*/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mpc_6axis.h"
#include "rnea_6axis.h"

#define MAX_STEPS 8000
#define NBLK 15   /* 每步每关节一个数的块数，顺序和 dump_loop.py 的 BLOCKS 一致 */
#define NM (2 * MPC_CS_NB)

typedef struct {
    float q[MPC_NJ], dq[MPC_NJ];          /* 控制器看到的（带噪声、延迟的）测量 */
    float qr_m[MPC_NJ], dqr_m[MPC_NJ];    /* 测量时刻的参考 */
    float qr_a[MPC_NJ], dqr_a[MPC_NJ], ddqr_a[MPC_NJ];        /* 力矩生效时刻的参考 */
    float qr_end[MPC_NJ], dqr_end[MPC_NJ], ddqr_end[MPC_NJ];  /* 时域末尾的参考 */
    float ff_ref[MPC_NJ];                 /* Python 的 tau_ff（含摩擦补偿） */
    float rho_ref[MPC_NJ];                /* Python 的惯量比 */
    float w_ref[MPC_NJ];                  /* Python 观测器更新后的 ŵ */
    float dtau_ref[MPC_NJ];               /* Python 的 Δτ = ρ·u（饱和前） */
    float tau_ref[MPC_NJ];                /* Python 的总力矩（饱和后） */
    float qr_e[MPC_CS_NB][MPC_NJ], dqr_e[MPC_CS_NB][MPC_NJ];  /* 检查点的参考 */
} Step;

static Step steps[MAX_STEPS];

static int rd(FILE *f, float *dst, int n)
{
    double v;
    for (int i = 0; i < n; ++i) {
        if (fscanf(f, "%lf", &v) != 1) return 0;
        dst[i] = (float)v;
    }
    return 1;
}

/* 相对误差，分母至少取 1，避免在 0 附近放大 */
static double rel(double a, double b)
{
    double d = fabs(a - b), m = fabs(b);
    return d / (m > 1.0 ? m : 1.0);
}

/* tref = RNEA(q, dq, ddq) + 摩擦补偿(dq)，参考前馈 */
static int tref(const float *q, const float *dq, const float *ddq, float *out)
{
    casadi_real cq[MPC_NJ], cdq[MPC_NJ], cddq[MPC_NJ], ctau[MPC_NJ];
    for (int j = 0; j < MPC_NJ; ++j) {
        cq[j] = (casadi_real)q[j]; cdq[j] = (casadi_real)dq[j];
        cddq[j] = (casadi_real)ddq[j]; ctau[j] = (casadi_real)0;
    }
    const casadi_real *arg[3] = {cq, cdq, cddq};
    casadi_real *res[1] = {ctau};
    if (rnea(arg, res, 0, 0, 0) != 0) return -1;
    float fric[MPC_NJ];
    mpc_friction_ff(dq, fric);
    for (int j = 0; j < MPC_NJ; ++j) out[j] = (float)ctau[j] + fric[j];
    return 0;
}

int main(int argc, char **argv)
{
    const char *path = (argc > 1) ? argv[1] : "loop_cases.txt";
    FILE *f = fopen(path, "r");
    if (!f) { fprintf(stderr, "打不开 %s\n", path); return 1; }

    int nj = 0, nu = 0, nb = 0, D = 0, nseg = 0;
    if (fscanf(f, "%d %d %d %d %d", &nj, &nu, &nb, &D, &nseg) != 5) {
        fprintf(stderr, "读文件头失败\n"); fclose(f); return 1;
    }
    if (nj != MPC_NJ || nu != MPC_CS_NU || nb != MPC_CS_NB || D != MPC_EST_D) {
        fprintf(stderr, "维度不符: 文件 nj=%d NU=%d NB=%d D=%d, 头文件 %d %d %d %d\n",
                nj, nu, nb, D, MPC_NJ, MPC_CS_NU, MPC_CS_NB, MPC_EST_D);
        fclose(f); return 1;
    }

    double w_ff = 0.0, w_ffe = 0.0, w_rho = 0.0, w_w = 0.0, w_tau = 0.0, w_dtau = 0.0;
    int wt_i = -1, wt_j = -1, wt_seg = -1, ww_i = -1, ww_j = -1;
    int n_effort_viol = 0, total = 0;
    double worst_effort_ov = 0.0;

    for (int sg = 0; sg < nseg; ++sg) {
        int i0 = 0, count = 0;
        if (fscanf(f, "%d %d", &i0, &count) != 2 || count > MAX_STEPS) {
            fprintf(stderr, "段 %d 头读取失败或步数超过 MAX_STEPS\n", sg); return 1;
        }
        /* 观测器和求解器热启动：从 Python 在段起点的状态拷过来 */
        for (int j = 0; j < MPC_NJ; ++j) {
            float zh[3 + MPC_EST_D + 1];
            if (!rd(f, zh, 3 + MPC_EST_D + 1)) { fprintf(stderr, "观测器初值读取失败\n"); return 1; }
            mpc_est_set(j, zh, zh + 3);
        }
        for (int j = 0; j < MPC_NJ; ++j) {
            float w[3 * MPC_CS_NU + 2 * NM];
            if (!rd(f, w, 3 * MPC_CS_NU + 2 * NM)) { fprintf(stderr, "热启动读取失败\n"); return 1; }
            mpc_cs_set_warm(j, w, w + MPC_CS_NU, w + 2 * MPC_CS_NU,
                            w + 3 * MPC_CS_NU, w + 3 * MPC_CS_NU + NM);
        }
        for (int i = 0; i < count; ++i) {
            Step *s = &steps[i];
            float *blocks[NBLK] = {s->q, s->dq, s->qr_m, s->dqr_m, s->qr_a, s->dqr_a,
                                   s->ddqr_a, s->qr_end, s->dqr_end, s->ddqr_end,
                                   s->ff_ref, s->rho_ref, s->w_ref, s->dtau_ref, s->tau_ref};
            int ok = 1;
            for (int b = 0; b < NBLK && ok; ++b) ok = rd(f, blocks[b], MPC_NJ);
            ok = ok && rd(f, &s->qr_e[0][0], MPC_CS_NB * MPC_NJ)
                    && rd(f, &s->dqr_e[0][0], MPC_CS_NB * MPC_NJ);
            if (!ok) { fprintf(stderr, "段 %d 第 %d 步字段不全\n", sg, i); return 1; }
        }

        for (int i = 0; i < count; ++i) {
            Step *s = &steps[i];

            /* 1. 前馈：RNEA(测量 q, 测量 dq, 生效时刻的参考 ddq) + 摩擦补偿 */
            float tau_ff[MPC_NJ], t0[MPC_NJ], t1[MPC_NJ], tau_ff_end[MPC_NJ];
            if (tref(s->q, s->dq, s->ddqr_a, tau_ff) != 0
                || tref(s->qr_a, s->dqr_a, s->ddqr_a, t0) != 0
                || tref(s->qr_end, s->dqr_end, s->ddqr_end, t1) != 0) {
                fprintf(stderr, "步 %d: rnea() 返回非零\n", i0 + i); return 1;
            }
            for (int j = 0; j < MPC_NJ; ++j) {
                tau_ff_end[j] = tau_ff[j] + (t1[j] - t0[j]);
                double e = fabs((double)tau_ff[j] - (double)s->ff_ref[j]);
                if (e > w_ff) w_ff = e;
                if (fabs(t1[j] - t0[j]) > w_ffe) w_ffe = fabs(t1[j] - t0[j]);
            }

            /* 2. P3 惯量比 */
            float rho[MPC_NJ];
            mpc_inertia_ratio(s->q, rho);
            for (int j = 0; j < MPC_NJ; ++j) {
                double e = rel(rho[j], s->rho_ref[j]);
                if (e > w_rho) w_rho = e;
            }

            /* 3. P1 观测器在测量时刻更新，所有关节 */
            for (int j = 0; j < MPC_NJ; ++j) {
                mpc_est_update(j, s->qr_m[j] - s->q[j], s->dqr_m[j] - s->dq[j]);
                double e = rel(mpc_est_w(j), s->w_ref[j]);
                if (e > w_w) { w_w = e; ww_i = i0 + i; ww_j = j; }
            }

            /* 4. P4: 每拍解全部关节 */
            float u[MPC_NJ];
            for (int j = 0; j < MPC_NJ; ++j) {
                float anchor[2], w_hat = 0.0f, qe[MPC_CS_NB], dqe[MPC_CS_NB];
                mpc_est_anchor(j, anchor, &w_hat);          /* P2 前推到生效时刻 */
                for (int k = 0; k < MPC_CS_NB; ++k) {
                    qe[k] = s->qr_e[k][j]; dqe[k] = s->dqr_e[k][j];
                }
                u[j] = mpc_cs_step(j, anchor, w_hat, tau_ff[j], tau_ff_end[j],
                                   rho[j], qe, dqe);
            }

            /* 5. Δτ = ρ·u，装配饱和 */
            float dtau[MPC_NJ], tau_cmd[MPC_NJ];
            for (int j = 0; j < MPC_NJ; ++j) dtau[j] = rho[j] * u[j];
            mpc_saturate_total(tau_ff, dtau, tau_cmd);
            /* 告诉观测器真正作用到对象上的量。回放测试必须喂 Python 实际发出去的 u：
             * 测量值是 Python 的命令产生的，喂 C 自己的命令会把 float32 差异
             * 当成"扰动"积分进观测器。板上就是 (tau_cmd - tau_ff)/rho。 */
            for (int j = 0; j < MPC_NJ; ++j)
                mpc_est_commit(j, (s->tau_ref[j] - s->ff_ref[j]) / s->rho_ref[j]);

            for (int j = 0; j < MPC_NJ; ++j) {
                double e = fabs((double)tau_cmd[j] - (double)s->tau_ref[j]);
                double ed = fabs((double)dtau[j] - (double)s->dtau_ref[j]);
                if (e > w_tau) { w_tau = e; wt_i = i0 + i; wt_j = j; wt_seg = sg; }
                if (ed > w_dtau) w_dtau = ed;
                if (fabsf(tau_cmd[j]) > MPC_EFFORT_MAX[j] * 1.0001f + 1e-4f) {
                    n_effort_viol++;
                    double ov = fabs((double)tau_cmd[j]) - MPC_EFFORT_MAX[j];
                    if (ov > worst_effort_ov) worst_effort_ov = ov;
                }
            }
        }
        printf("段 %d: 步 [%d, %d)\n", sg, i0, i0 + count);
        total += count;
    }
    fclose(f);

    printf("回放 %d 步, 每拍解全部 %d 个关节, 定迭代 %d 次, 延迟补偿 D=%d\n",
           total, MPC_NJ, MPC_CS_ITERS, MPC_EST_D);
    printf("tau_ff 最大偏差      %.4g Nm   (时域内前馈变化最大 %.3g Nm)\n", w_ff, w_ffe);
    printf("惯量比 ρ 最大相对差  %.3g   (P3)\n", w_rho);
    printf("扰动 ŵ 最大相对差    %.3g", w_w);
    if (ww_i >= 0) printf("   (步 %d, 关节 %d)  (P1)", ww_i, ww_j + 1);
    printf("\n");
    printf("Δτ 最大偏差          %.4g Nm\n", w_dtau);
    printf("总力矩最大偏差       %.4g Nm", w_tau);
    if (wt_i >= 0) printf("   (段 %d 步 %d, 关节 %d)", wt_seg, wt_i, wt_j + 1);
    printf("\n");
    printf("总力矩越限           %d   (最大超出 %.4g Nm)\n", n_effort_viol, worst_effort_ov);

    double lim = (double)MPC_DTAU_MAX[0];
    for (int j = 1; j < MPC_NJ; ++j)
        if ((double)MPC_DTAU_MAX[j] < lim) lim = MPC_DTAU_MAX[j];
    const double tol = 0.02 * lim;
    printf("判据: 力矩偏差 <= 2%% * min(DTAU_MAX) = %.4g Nm; ρ、ŵ 相对差 <= 1e-3\n", tol);

    const int fail = (n_effort_viol > 0) || (w_ff > tol) || (w_tau > tol)
                     || (w_rho > 1e-3) || (w_w > 1e-3);
    printf("\n%s\n", fail ? "FAIL"
           : "PASS  (前馈、惯量比、观测器、每拍全解、约束、装配逐步和 Python 一致; "
             "总力矩不超 EFFORT_MAX)");
    return fail ? 1 : 0;
}
