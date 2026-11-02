/* C 侧 float32 数值回归：喂 dump_qp.py 导出的求解输入，比对 mpc_cs_step() 的 Δτ。
 *
 * 两种回放（见 dump_qp.py）：
 *   逐条  每条都先 mpc_cs_set_warm() 拷 Python 的热启动状态，只验单次求解
 *   连续  每段第一条拷状态，之后用 C 自己的状态，验 float32 误差会不会累积
 * 另外单独比对：钳位上限、控制量边界（mpc_cs_control_bounds）。
 *
 * 编译见同目录 build.sh。*/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "mpc_6axis.h"

#define MAX_CASES 20000
#define NM (2 * MPC_CS_NB)

typedef struct {
    int   seg, joint;
    float ae_raw[2], ae_ref[2];   /* 钳位前 / Python 钳位后的锚点 */
    float w_hat, tau_ff, tau_ff_end, rho;
    float qr_e[MPC_CS_NB], dqr_e[MPC_CS_NB];
    float u[MPC_CS_NU], zu[MPC_CS_NU], yu[MPC_CS_NU], zs[NM], ys[NM];  /* 热启动 */
    float lo_u[MPC_CS_NU], hi_u[MPC_CS_NU];                           /* Python 的边界 */
    float u_ref;                  /* Python float64 的解（u 单位） */
} Case;

static Case cases[MAX_CASES];

static int rd(FILE *f, float *dst, int n)
{
    double v;
    for (int i = 0; i < n; ++i) {
        if (fscanf(f, "%lf", &v) != 1) return 0;
        dst[i] = (float)v;
    }
    return 1;
}

typedef struct { double worst_rel, worst_abs, sum_abs; int worst_i; float worst_c; } Stat;

static void stat_add(Stat *s, int i, float dtau_c, double dtau_py, int joint)
{
    double e = fabs((double)dtau_c - dtau_py);
    s->sum_abs += e;
    if (e > s->worst_abs) { s->worst_abs = e; s->worst_c = dtau_c; }
    /* 判据按各关节自己的 DTAU_MAX 取相对值：修正权限相差 10 倍，
     * 用全局最大值会放过手腕上的大相对偏差。 */
    double rel = e / (double)MPC_DTAU_MAX[joint];
    if (rel > s->worst_rel) { s->worst_rel = rel; s->worst_i = i; }
}

int main(int argc, char **argv)
{
    const char *path = (argc > 1) ? argv[1] : "qp_cases.txt";
    FILE *f = fopen(path, "r");
    if (!f) { fprintf(stderr, "打不开 %s\n", path); return 1; }

    int nj = 0, nu = 0, nb = 0, count = 0;
    if (fscanf(f, "%d %d %d %d", &nj, &nu, &nb, &count) != 4) {
        fprintf(stderr, "读文件头失败\n"); fclose(f); return 1;
    }
    if (nj != MPC_NJ || nu != MPC_CS_NU || nb != MPC_CS_NB) {
        fprintf(stderr, "维度不符: 文件 nj=%d NU=%d NB=%d, 头文件 %d %d %d\n",
                nj, nu, nb, MPC_NJ, MPC_CS_NU, MPC_CS_NB);
        fclose(f); return 1;
    }
    if (count > MAX_CASES) {
        fprintf(stderr, "用例数 %d 超过 MAX_CASES=%d\n", count, MAX_CASES);
        fclose(f); return 1;
    }
    for (int i = 0; i < count; ++i) {
        Case *c = &cases[i];
        float sc[4];
        if (fscanf(f, "%d %d", &c->seg, &c->joint) != 2
            || !rd(f, c->ae_raw, 2) || !rd(f, c->ae_ref, 2) || !rd(f, sc, 4)
            || !rd(f, c->qr_e, MPC_CS_NB) || !rd(f, c->dqr_e, MPC_CS_NB)
            || !rd(f, c->u, MPC_CS_NU) || !rd(f, c->zu, MPC_CS_NU) || !rd(f, c->yu, MPC_CS_NU)
            || !rd(f, c->zs, NM) || !rd(f, c->ys, NM)
            || !rd(f, c->lo_u, MPC_CS_NU) || !rd(f, c->hi_u, MPC_CS_NU)
            || !rd(f, &c->u_ref, 1)) {
            fprintf(stderr, "第 %d 条记录字段不全\n", i); fclose(f); return 1;
        }
        c->w_hat = sc[0]; c->tau_ff = sc[1]; c->tau_ff_end = sc[2]; c->rho = sc[3];
    }
    fclose(f);

    int n_clamp_mismatch = 0, n_clamped = 0, n_bound_mismatch = 0;
    int n_effort_viol = 0, n_box_viol = 0;
    float worst_clamp = 0.0f, worst_bound = 0.0f;
    double worst_effort_ov = 0.0;
    Stat strict = {0}, chain = {0};
    strict.worst_i = chain.worst_i = -1;

    /* ---- 逐条 ---- */
    for (int i = 0; i < count; ++i) {
        Case *c = &cases[i];
        const int j = c->joint;

        float ae_c[2];
        mpc_clamp_anchor(j, c->ae_raw, ae_c);
        for (int k = 0; k < 2; ++k) {
            if (c->ae_raw[k] != c->ae_ref[k]) n_clamped++;
            float d = fabsf(ae_c[k] - c->ae_ref[k]);
            if (d > worst_clamp) worst_clamp = d;
            if (d > 1e-5f * (1.0f + fabsf(c->ae_ref[k]))) n_clamp_mismatch++;
        }

        float lo[MPC_CS_NU], hi[MPC_CS_NU];
        mpc_cs_control_bounds(j, c->tau_ff, c->tau_ff_end, c->rho, lo, hi);
        for (int k = 0; k < MPC_CS_NU; ++k) {
            float d1 = fabsf(lo[k] - c->lo_u[k]), d2 = fabsf(hi[k] - c->hi_u[k]);
            float d = d1 > d2 ? d1 : d2;
            if (d > worst_bound) worst_bound = d;
            if (d > 1e-5f * (1.0f + fabsf(c->hi_u[k]))) n_bound_mismatch++;
        }

        mpc_cs_set_warm(j, c->u, c->zu, c->yu, c->zs, c->ys);
        float uc = mpc_cs_step(j, c->ae_raw, c->w_hat, c->tau_ff, c->tau_ff_end,
                               c->rho, c->qr_e, c->dqr_e);
        if (uc < lo[0] - 1e-7f || uc > hi[0] + 1e-7f) n_box_viol++;
        float dtau = c->rho * uc;
        /* 总力矩约束必须守住：这是负责人明确要求的硬约束 */
        if (fabsf(c->tau_ff + dtau) > MPC_EFFORT_MAX[j] * 1.0001f + 1e-4f) {
            n_effort_viol++;
            double ov = fabs((double)(c->tau_ff + dtau)) - MPC_EFFORT_MAX[j];
            if (ov > worst_effort_ov) worst_effort_ov = ov;
        }
        stat_add(&strict, i, dtau, (double)c->rho * (double)c->u_ref, j);
    }

    /* ---- 连续 ---- */
    int last_seg[MPC_NJ];
    for (int j = 0; j < MPC_NJ; ++j) last_seg[j] = -1;
    mpc_cs_reset();
    for (int i = 0; i < count; ++i) {
        Case *c = &cases[i];
        const int j = c->joint;
        if (c->seg != last_seg[j]) {
            mpc_cs_set_warm(j, c->u, c->zu, c->yu, c->zs, c->ys);
            last_seg[j] = c->seg;
        }
        float uc = mpc_cs_step(j, c->ae_raw, c->w_hat, c->tau_ff, c->tau_ff_end,
                               c->rho, c->qr_e, c->dqr_e);
        stat_add(&chain, i, c->rho * uc, (double)c->rho * (double)c->u_ref, j);
    }

    printf("用例数            %d   (钳位触发 %d 个分量), 定迭代 %d 次\n",
           count, n_clamped, MPC_CS_ITERS);
    printf("钳位不一致        %d   (最大偏差 %.3g)\n", n_clamp_mismatch, (double)worst_clamp);
    printf("控制量边界不一致  %d   (最大偏差 %.3g)\n", n_bound_mismatch, (double)worst_bound);
    const char *tag[2] = {"逐条", "连续"};
    Stat *st[2] = {&strict, &chain};
    for (int m = 0; m < 2; ++m) {
        printf("[%s] Δτ 平均偏差 %.3g Nm, 最大 %.3g Nm, 最大相对 %.3g%% of 本关节 DTAU_MAX",
               tag[m], st[m]->sum_abs / (count ? count : 1), st[m]->worst_abs,
               100.0 * st[m]->worst_rel);
        if (st[m]->worst_i >= 0)
            printf("   (用例 %d, 关节 %d)", st[m]->worst_i, cases[st[m]->worst_i].joint + 1);
        printf("\n");
    }
    printf("首个控制量出箱    %d\n", n_box_viol);
    printf("总力矩越限        %d   (最大超出 %.4g Nm)\n", n_effort_viol, worst_effort_ov);

    int fail = (n_clamp_mismatch > 0) || (n_bound_mismatch > 0) || (n_box_viol > 0)
               || (strict.worst_rel > 0.005) || (chain.worst_rel > 0.02)
               || (n_effort_viol > 0);
    printf("\n%s\n", fail ? "FAIL" : "PASS  (判据: 钳位、边界一致; 逐条 Δτ 偏差 <= 0.5%, "
                                     "连续 <= 2% 本关节 DTAU_MAX; 控制量不出箱; "
                                     "|tau_ff+Δτ| 不超 EFFORT_MAX)");
    return fail ? 1 : 0;
}
