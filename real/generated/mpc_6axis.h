#ifndef MPC_6AXIS_H
#define MPC_6AXIS_H
#ifdef __cplusplus
extern "C" {
#endif

#define MPC_NJ  6
#define MPC_N   10
#define MPC_NZ  30
#define MPC_NC  30
#define MPC_BASE_DT  0.002f
#define MPC_DT  0.006f
#define MPC_JOINTS_PER_TICK  2
#define MPC_DTAU_MAX  1.20000000e+01f
#define MPC_MAX_ITER  100

/* 锚点误差：e_anchor,k = x_ref(t0) - x(t0+k*DT)。
 * 参考偏移 delta_k = x_ref(t0) - x_ref(t0+k*DT)，
 * 因此 e_anchor,k - delta_k 是未来时刻的普通跟踪误差。
 * 每个关节使用独立、固定 Ad/Bd 的 OSQP workspace。 */
extern const float MPC_A10[MPC_NJ];
extern const float MPC_B1[MPC_NJ];

/* 锚点误差钳位上限（factor=3）。
 * 误差远超线性范围时 QP 的答案必然是顶满 DTAU_MAX，但这种输入会让全部 MPC_N 个
 * 控制约束同时激活，正是 ADMM 最慢的工况——不钳位时最坏需要 1000 次迭代，
 * 装不进 1 ms 的 tick。钳位后最坏 55 次。
 * mpc_build_vectors() 内部已自动钳位，调用方无需自己处理。 */
extern const float MPC_E_MAX[MPC_NJ];
extern const float MPC_DE_MAX[MPC_NJ];

void mpc_get_AB(int j, float *A, float *B);
void mpc_matvec2(const float *A, const float *v, float *out);
/* 把锚点误差钳到 [-MPC_E_MAX, MPC_E_MAX] x [-MPC_DE_MAX, MPC_DE_MAX]。
 * out 可以和 in 指向同一块内存。 */
void mpc_clamp_anchor(int j, const float in[2], float out[2]);

/* 构造一个关节的在线 QP 向量：
 * anchor_error = [q_ref(t0)-q(t0), dq_ref(t0)-dq(t0)]；
 * delta_seq[k] = x_ref(t0)-x_ref(t0+k*DT)，共 MPC_N+1 个点。
 * q_out、l_out、u_out 的长度分别为 MPC_NZ、MPC_NC、MPC_NC。 */
void mpc_build_vectors(int j, const float anchor_error[2],
                       const float delta_seq[MPC_N + 1][2],
                       float q_out[MPC_NZ], float l_out[MPC_NC],
                       float u_out[MPC_NC]);

/* 更新指定关节的线性项 q 和上下界 l/u。 */
int mpc_update_joint(int j, const float *q, const float *l, const float *u);
/* 求解指定关节的 QP；返回 OSQP API 错误码（0 表示调用成功）。 */
int mpc_solve_joint(int j);
/* 读取指定关节最近一次求解的 OSQP 状态码（1=solved，2=solved inaccurate）。 */
int mpc_get_status(int j);
/* 读取指定关节解中的第一个 Δτ，已钳到 ±MPC_DTAU_MAX。失败或索引非法时返回 0。
 *
 * 必须钳位：ADMM 的中间迭代点不满足约束，max_iter 用尽时取回的值可能越界。
 * 调用方的处置策略要和仿真一致（USE_UNCONVERGED_ITERATE=True）：
 *   mpc_get_status(j) 返回 1(solved) 或 2(solved inaccurate) 时直接采用；
 *   其余情况仿真里也采用当前迭代点（大扰动下 hold 过期值更糟）。 */
float mpc_get_first_dtau(int j);

#ifdef __cplusplus
}
#endif
#endif
