/* C 侧 float32 RNEA 回归：生成的 rnea_6axis.c 对 pinocchio float64。
 *
 * 前馈是输出力矩的主项，这里验的是它没有结构性错误（模型/关节顺序/重力符号），
 * 以及 float32 的精度损失远小于 MPC 的修正权限 DTAU_MAX。
 *
 * 编译见同目录 build.sh。*/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mpc_6axis.h"
#include "rnea_6axis.h"

#define MAX_NJ     8
#define MAX_CASES  6000
#define MAX_GROUPS 8
#define NAME_LEN   16

/* 判据：前馈误差对 MPC 而言等同外部扰动，必须远小于修正权限。 */
#define TOL_FRACTION_OF_DTAU  0.01f

typedef struct {
    int    group;
    double q[MAX_NJ], dq[MAX_NJ], ddq[MAX_NJ], tau_ref[MAX_NJ];
} Case;

typedef struct {
    char   name[NAME_LEN];
    int    count;
    double worst_abs;
    double worst_rel;
    double sum_abs;
    int    worst_case;
    int    worst_joint;
} Group;

static Case  cases[MAX_CASES];
static Group groups[MAX_GROUPS];

int main(int argc, char **argv)
{
    const char *path = (argc > 1) ? argv[1] : "rnea_cases.txt";
    FILE *f = fopen(path, "r");
    if (!f) { fprintf(stderr, "打不开 %s\n", path); return 1; }

    int n = 0, count = 0, ngroups = 0;
    if (fscanf(f, "%d %d %d", &n, &count, &ngroups) != 3) {
        fprintf(stderr, "读文件头失败\n"); fclose(f); return 1;
    }
    if (n != MPC_NJ || n > MAX_NJ) {
        fprintf(stderr, "自由度不符: 文件 n=%d, MPC_NJ=%d\n", n, MPC_NJ);
        fclose(f); return 1;
    }
    if (count > MAX_CASES || ngroups > MAX_GROUPS) {
        fprintf(stderr, "用例数 %d 或分组数 %d 超上限\n", count, ngroups);
        fclose(f); return 1;
    }
    for (int g = 0; g < ngroups; ++g) {
        if (fscanf(f, "%15s", groups[g].name) != 1) {
            fprintf(stderr, "读分组名失败\n"); fclose(f); return 1;
        }
        groups[g].count = 0; groups[g].worst_abs = 0.0;
        groups[g].worst_rel = 0.0; groups[g].sum_abs = 0.0;
        groups[g].worst_case = -1; groups[g].worst_joint = -1;
    }

    for (int i = 0; i < count; ++i) {
        Case *c = &cases[i];
        if (fscanf(f, "%d", &c->group) != 1) goto bad;
        if (c->group < 0 || c->group >= ngroups) {
            fprintf(stderr, "第 %d 条记录分组号越界\n", i); fclose(f); return 1;
        }
        double *blocks[4] = { c->q, c->dq, c->ddq, c->tau_ref };
        for (int b = 0; b < 4; ++b)
            for (int j = 0; j < n; ++j)
                if (fscanf(f, "%lf", &blocks[b][j]) != 1) goto bad;
        continue;
bad:
        fprintf(stderr, "第 %d 条记录字段不全\n", i); fclose(f); return 1;
    }
    fclose(f);

    const double tol = TOL_FRACTION_OF_DTAU * (double)MPC_DTAU_MAX;
    double overall_worst_abs = 0.0, overall_worst_rel = 0.0;
    int n_over_tol = 0, n_nonfinite = 0;

    for (int i = 0; i < count; ++i) {
        Case *c = &cases[i];
        casadi_real q[MAX_NJ], dq[MAX_NJ], ddq[MAX_NJ], tau[MAX_NJ];
        for (int j = 0; j < n; ++j) {
            q[j]   = (casadi_real)c->q[j];
            dq[j]  = (casadi_real)c->dq[j];
            ddq[j] = (casadi_real)c->ddq[j];
            tau[j] = (casadi_real)0;
        }
        const casadi_real *arg[3] = { q, dq, ddq };
        casadi_real *res[1] = { tau };
        if (rnea(arg, res, 0, 0, 0) != 0) {
            fprintf(stderr, "用例 %d: rnea() 返回非零\n", i); return 1;
        }

        Group *g = &groups[c->group];
        g->count++;
        for (int j = 0; j < n; ++j) {
            if (!isfinite((double)tau[j])) { n_nonfinite++; continue; }
            double e   = fabs((double)tau[j] - c->tau_ref[j]);
            double ref = fabs(c->tau_ref[j]);
            double rel = e / (ref > 1.0 ? ref : 1.0);
            g->sum_abs += e;
            if (e > g->worst_abs) {
                g->worst_abs = e; g->worst_case = i; g->worst_joint = j;
            }
            if (rel > g->worst_rel) g->worst_rel = rel;
            if (e > overall_worst_abs) overall_worst_abs = e;
            if (rel > overall_worst_rel) overall_worst_rel = rel;
            if (e > tol) n_over_tol++;
        }
    }

    printf("用例 %d 个, 自由度 %d, 判据 = %.1f%% * DTAU_MAX = %.4g Nm\n",
           count, n, 100.0 * TOL_FRACTION_OF_DTAU, tol);
    printf("\n%-10s %6s %14s %14s %12s   %s\n",
           "分组", "用例", "平均|误差|", "最大|误差|", "最大相对", "最坏点");
    for (int g = 0; g < ngroups; ++g) {
        Group *G = &groups[g];
        int ne = G->count * n;
        printf("%-10s %6d %14.3e %14.3e %12.2e   ",
               G->name, G->count, ne ? G->sum_abs / ne : 0.0,
               G->worst_abs, G->worst_rel);
        if (G->worst_case >= 0) {
            Case *c = &cases[G->worst_case];
            printf("例%d 关节%d  Py=%.4f  max|dq|=",
                   G->worst_case, G->worst_joint + 1,
                   c->tau_ref[G->worst_joint]);
            double m = 0.0;
            for (int j = 0; j < n; ++j)
                if (fabs(c->dq[j]) > m) m = fabs(c->dq[j]);
            printf("%.1f", m);
        }
        printf("\n");
    }

    printf("\n非有限输出     %d\n", n_nonfinite);
    printf("超判据的分量   %d\n", n_over_tol);
    printf("全局最大|误差| %.4g Nm  (%.3g%% of DTAU_MAX)\n",
           overall_worst_abs, 100.0 * overall_worst_abs / (double)MPC_DTAU_MAX);
    printf("全局最大相对   %.3g\n", overall_worst_rel);

    /* 极端组允许超绝对判据（dq=130 rad/s 不是真实工况，力矩本身就到几百 Nm），
     * 但相对精度必须守住，否则说明不是舍入而是算法问题。 */
    int fail = (n_nonfinite > 0) || (overall_worst_rel > 1.0e-3);
    for (int g = 0; g < ngroups; ++g) {
        if (strcmp(groups[g].name, "extreme") == 0) continue;
        if (groups[g].worst_abs > tol) fail = 1;
    }
    printf("\n%s\n", fail ? "FAIL"
           : "PASS  (判据: 输出有限; gravity/sim 组最大误差 <= 1% DTAU_MAX; "
             "全局相对误差 <= 1e-3)");
    return fail ? 1 : 0;
}
