#include <math.h>

#include "mpc_6axis.h"
#include "osqp_api_functions.h"

#include "generated_solver/joint0/mpc_j0workspace.h"
#include "generated_solver/joint1/mpc_j1workspace.h"
#include "generated_solver/joint2/mpc_j2workspace.h"
#include "generated_solver/joint3/mpc_j3workspace.h"
#include "generated_solver/joint4/mpc_j4workspace.h"
#include "generated_solver/joint5/mpc_j5workspace.h"

/* 每个关节的离散模型系数（q=0 处线性化）。 */
const float MPC_A10[MPC_NJ] = { 0.00000000e+00f, 0.00000000e+00f, 0.00000000e+00f, 0.00000000e+00f, 0.00000000e+00f, 0.00000000e+00f };
const float MPC_B1[MPC_NJ] = { -2.00000000e-01f, -2.53377049e-01f, -6.16131148e-01f, -5.00000000e-01f, -4.23344262e-01f, -9.00000000e-01f };

/* 锚点误差钳位上限（factor=3）。由 LQR 增益反推：
 * K_pos = [62.01, 52.14, 23.89, 29.06, 33.82, 16.58]
 * K_vel = [4.59, 3.83, 1.73, 2.11, 2.46, 1.2 ] */
const float MPC_E_MAX[MPC_NJ] = { 5.80529571e-01f, 6.90476355e-01f, 1.50704616e+00f, 1.23899570e+00f, 1.06434733e+00f, 2.17115614e+00f };
const float MPC_DE_MAX[MPC_NJ] = { 7.84431700e+00f, 9.40909944e+00f, 2.08166433e+01f, 1.70872215e+01f, 1.46526657e+01f, 3.00395580e+01f };

static float mpc_clamp1(float v, float lim)
{
    if (v >  lim) return  lim;
    if (v < -lim) return -lim;
    return v;
}

void mpc_clamp_anchor(int j, const float in[2], float out[2])
{
    if (j < 0 || j >= MPC_NJ || in == 0 || out == 0) return;
    out[0] = mpc_clamp1(in[0], MPC_E_MAX[j]);
    out[1] = mpc_clamp1(in[1], MPC_DE_MAX[j]);
}

static OSQPSolver *const MPC_SOLVERS[MPC_NJ] = {
    &mpc_j0solver,
    &mpc_j1solver,
    &mpc_j2solver,
    &mpc_j3solver,
    &mpc_j4solver,
    &mpc_j5solver
};

void mpc_get_AB(int j, float *A, float *B)
{
    if (j < 0 || j >= MPC_NJ || A == 0) return;
    A[0] = 1.0f; A[1] = MPC_DT;
    A[2] = MPC_A10[j]; A[3] = 1.0f;
    if (B != 0) {
        B[0] = 0.0f; B[1] = MPC_B1[j];
    }
}

void mpc_matvec2(const float *A, const float *v, float *out)
{
    if (A == 0 || v == 0 || out == 0) return;
    out[0] = A[0]*v[0] + A[1]*v[1];
    out[1] = A[2]*v[0] + A[3]*v[1];
}

void mpc_build_vectors(int j, const float anchor_error[2],
                       const float delta_seq[MPC_N + 1][2],
                       float q_out[MPC_NZ], float l_out[MPC_NC],
                       float u_out[MPC_NC])
{
    if (j < 0 || j >= MPC_NJ || anchor_error == 0 || delta_seq == 0 ||
        q_out == 0 || l_out == 0 || u_out == 0) return;
    float A[4];
    mpc_get_AB(j, A, 0);
    /* 钳位放在这里而不是交给调用方，这样 C 侧不可能漏掉、和仿真必然一致。 */
    float ae[2];
    mpc_clamp_anchor(j, anchor_error, ae);
    for (int k = 0; k < MPC_N; ++k) {
        q_out[2*k]     = -2.00000000e+00f * 2.00000000e+03f * delta_seq[k + 1][0];
        q_out[2*k + 1] = -2.00000000e+00f * 5.00000000e+00f * delta_seq[k + 1][1];
    }
    /* 力矩修正量的线性代价项为零，必须显式写入整个控制段。 */
    for (int k = 0; k < MPC_N; ++k) {
        q_out[2*MPC_N + k] = 0.0f;
    }
    for (int k = 0; k < MPC_N; ++k) {
        const float ax0 = A[0]*delta_seq[k][0] + A[1]*delta_seq[k][1];
        const float ax1 = A[2]*delta_seq[k][0] + A[3]*delta_seq[k][1];
        float rhs0 = delta_seq[k + 1][0] - ax0;
        float rhs1 = delta_seq[k + 1][1] - ax1;
        if (k == 0) {
            rhs0 += A[0]*ae[0] + A[1]*ae[1];
            rhs1 += A[2]*ae[0] + A[3]*ae[1];
        }
        l_out[2*k] = u_out[2*k] = rhs0;
        l_out[2*k + 1] = u_out[2*k + 1] = rhs1;
    }
    for (int k = 0; k < MPC_N; ++k) {
        l_out[2*MPC_N + k] = -1.20000000e+01f;
        u_out[2*MPC_N + k] =  1.20000000e+01f;
    }
}

int mpc_update_joint(int j, const float *q, const float *l, const float *u)
{
    if (j < 0 || j >= MPC_NJ || MPC_SOLVERS[j] == 0 ||
        MPC_SOLVERS[j]->work == 0 || q == 0 || l == 0 || u == 0) return -1;
    return (int)osqp_update_data_vec(MPC_SOLVERS[j], q, l, u);
}

int mpc_solve_joint(int j)
{
    if (j < 0 || j >= MPC_NJ || MPC_SOLVERS[j] == 0) return -1;
    return (int)osqp_solve(MPC_SOLVERS[j]);
}

int mpc_get_status(int j)
{
    if (j < 0 || j >= MPC_NJ || MPC_SOLVERS[j] == 0 ||
        MPC_SOLVERS[j]->info == 0) return -1;
    return (int)MPC_SOLVERS[j]->info->status_val;
}

float mpc_get_first_dtau(int j)
{
    if (j < 0 || j >= MPC_NJ || MPC_SOLVERS[j] == 0 ||
        MPC_SOLVERS[j]->solution == 0 ||
        MPC_SOLVERS[j]->solution->x == 0) return 0.0f;
    const float v = (float)MPC_SOLVERS[j]->solution->x[2 * MPC_N];
    if (!isfinite(v)) return 0.0f;
    return mpc_clamp1(v, MPC_DTAU_MAX);
}
