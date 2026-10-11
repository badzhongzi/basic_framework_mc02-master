/* C 侧 float32 数值回归：喂 dump_qp.py 导出的 QP 输入，比对 Δτ。
 *
 * 按记录顺序回放，这样每个关节看到的求解序列和仿真一致，热启动状态可比。
 * 同时校验钳位：C 的 mpc_clamp_anchor() 必须和 Python 钳出同一个值。
 *
 * 编译见同目录 build.sh。*/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "mpc_6axis.h"

#define MAX_CASES 20000

typedef struct {
    int   joint;
    float ae_raw[2];
    float ae_ref[2];              /* Python 钳位后的锚点误差 */
    float delta[MPC_N + 1][2];
    float dtau_ref;               /* Python float64 求得的 Δτ */
} Case;

static Case cases[MAX_CASES];

int main(int argc, char **argv)
{
    const char *path = (argc > 1) ? argv[1] : "qp_cases.txt";
    FILE *f = fopen(path, "r");
    if (!f) { fprintf(stderr, "打不开 %s\n", path); return 1; }

    int nj = 0, n = 0, count = 0;
    if (fscanf(f, "%d %d %d", &nj, &n, &count) != 3) {
        fprintf(stderr, "读文件头失败\n"); fclose(f); return 1;
    }
    if (nj != MPC_NJ || n != MPC_N) {
        fprintf(stderr, "维度不符: 文件 nj=%d N=%d, 头文件 MPC_NJ=%d MPC_N=%d\n",
                nj, n, MPC_NJ, MPC_N);
        fclose(f); return 1;
    }
    if (count > MAX_CASES) {
        fprintf(stderr, "用例数 %d 超过 MAX_CASES=%d\n", count, MAX_CASES);
        fclose(f); return 1;
    }

    for (int i = 0; i < count; ++i) {
        Case *c = &cases[i];
        double v;
        if (fscanf(f, "%d", &c->joint) != 1) {
            fprintf(stderr, "第 %d 条记录读取失败\n", i); fclose(f); return 1;
        }
        for (int k = 0; k < 2; ++k) { if (fscanf(f, "%lf", &v) != 1) goto bad; c->ae_raw[k] = (float)v; }
        for (int k = 0; k < 2; ++k) { if (fscanf(f, "%lf", &v) != 1) goto bad; c->ae_ref[k] = (float)v; }
        for (int k = 0; k <= MPC_N; ++k)
            for (int m = 0; m < 2; ++m) {
                if (fscanf(f, "%lf", &v) != 1) goto bad;
                c->delta[k][m] = (float)v;
            }
        if (fscanf(f, "%lf", &v) != 1) goto bad;
        c->dtau_ref = (float)v;
        continue;
bad:
        fprintf(stderr, "第 %d 条记录字段不全\n", i); fclose(f); return 1;
    }
    fclose(f);

    float q[MPC_NZ], l[MPC_NC], u[MPC_NC];
    double worst_abs = 0.0, sum_abs = 0.0;
    int worst_i = -1, n_bad_status = 0, n_clamp_mismatch = 0, n_clamped = 0;
    float worst_clamp = 0.0f, worst_c = 0.0f;

    for (int i = 0; i < count; ++i) {
        Case *c = &cases[i];

        /* 钳位一致性：C 必须和 Python 钳到同一个值 */
        float ae_c[2];
        mpc_clamp_anchor(c->joint, c->ae_raw, ae_c);
        for (int k = 0; k < 2; ++k) {
            if (c->ae_raw[k] != c->ae_ref[k]) n_clamped++;
            float d = fabsf(ae_c[k] - c->ae_ref[k]);
            if (d > worst_clamp) worst_clamp = d;
            /* float32 往返只应有舍入级差异 */
            if (d > 1e-5f * (1.0f + fabsf(c->ae_ref[k]))) n_clamp_mismatch++;
        }

        mpc_build_vectors(c->joint, c->ae_raw, c->delta, q, l, u);
        if (mpc_update_joint(c->joint, q, l, u) != 0) {
            fprintf(stderr, "用例 %d: mpc_update_joint 失败\n", i); return 1;
        }
        mpc_solve_joint(c->joint);
        int st = mpc_get_status(c->joint);
        if (st != 1 && st != 2) n_bad_status++;

        float dtau = mpc_get_first_dtau(c->joint);
        double e = fabs((double)dtau - (double)c->dtau_ref);
        sum_abs += e;
        if (e > worst_abs) { worst_abs = e; worst_i = i; worst_c = dtau; }
    }

    printf("用例数            %d   (钳位触发 %d 个分量)\n", count, n_clamped);
    printf("异常 status       %d\n", n_bad_status);
    printf("钳位不一致        %d   (最大偏差 %.3g)\n", n_clamp_mismatch, (double)worst_clamp);
    printf("Δτ 平均绝对偏差   %.6g Nm\n", sum_abs / (count ? count : 1));
    printf("Δτ 最大绝对偏差   %.6g Nm", worst_abs);
    if (worst_i >= 0)
        printf("   (用例 %d, 关节 %d, C=%.6f Py=%.6f)",
               worst_i, cases[worst_i].joint + 1,
               (double)worst_c, (double)cases[worst_i].dtau_ref);
    printf("\n");
    printf("相对 DTAU_MAX=%.1f Nm 为 %.3g%%\n",
           (double)MPC_DTAU_MAX, 100.0 * worst_abs / (double)MPC_DTAU_MAX);

    int fail = (n_bad_status > 0) || (n_clamp_mismatch > 0)
               || (worst_abs > 0.02 * (double)MPC_DTAU_MAX);
    printf("\n%s\n", fail ? "FAIL" : "PASS  (判据: 无异常 status, 钳位一致, "
                                     "Δτ 最大偏差 <= 2% DTAU_MAX)");
    return fail ? 1 : 0;
}
