/*
 * ec_traj 主机回归测试。
 *
 * 编译 (mingw):
 *   gcc -std=c99 -O2 -Wall -Wextra -I Engineer_Components_master/include \
 *       tests/traj/test_traj_host.c Engineer_Components_master/src/ec_traj.c \
 *       -lm -o tests/traj/test_traj_host.exe
 *
 * 覆盖点: 边界条件、换计划时的 C2 连续性、超出末尾后的保持、越限时的时长
 * 拉伸、目标越限拒绝、多节点拼接连续性、刹车、双缓冲发布、tick 回绕。
 */
#include "ec_traj.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define N_JOINT 6u

static int g_fail;
static int g_checks;

static void expect(int cond, const char *what) {
  ++g_checks;
  if (!cond) {
    ++g_fail;
    printf("  FAIL: %s\n", what);
  }
}

static void expect_near(float got, float want, float tol, const char *what) {
  ++g_checks;
  if (!(fabsf(got - want) <= tol)) {
    ++g_fail;
    printf("  FAIL: %s (got %.9g, want %.9g, tol %.3g)\n", what, (double)got,
           (double)want, (double)tol);
  }
}

static void limits_default(EC_TrajLimits *lim) {
  unsigned j;
  ec_traj_limits_init(lim, N_JOINT);
  for (j = 0; j < N_JOINT; ++j) {
    lim->q_min[j] = -3.14f;
    lim->q_max[j] = 3.14f;
    lim->dq_max[j] = 2.0f;
    lim->ddq_max[j] = 20.0f;
  }
  lim->min_duration = 0.02f;
  lim->max_duration = 10.0f;
}

static void state_set(EC_JointStateReference *s, float q, float dq, float ddq) {
  unsigned j;
  memset(s, 0, sizeof(*s));
  s->joint_count = N_JOINT;
  for (j = 0; j < N_JOINT; ++j) {
    s->q[j] = q;
    s->q_dot[j] = dq;
    s->q_ddot[j] = ddq;
  }
}

/* 在 [0, total_duration] 上密集采样, 取各关节的峰值和位置极值。 */
typedef struct {
  float dq_peak;
  float ddq_peak;
  float q_lo;
  float q_hi;
} Scan;

static void scan_plan(const EC_TrajPlan *plan, Scan *out) {
  const int steps = 4000;
  int k;
  unsigned j;
  out->dq_peak = 0.0f;
  out->ddq_peak = 0.0f;
  out->q_lo = 1.0e9f;
  out->q_hi = -1.0e9f;
  for (k = 0; k <= steps; ++k) {
    EC_JointStateReference s;
    const float t = plan->total_duration * (float)k / (float)steps;
    if (!ec_traj_eval_time(plan, t, &s)) {
      expect(0, "scan_plan: eval failed");
      return;
    }
    for (j = 0; j < N_JOINT; ++j) {
      if (fabsf(s.q_dot[j]) > out->dq_peak) out->dq_peak = fabsf(s.q_dot[j]);
      if (fabsf(s.q_ddot[j]) > out->ddq_peak) out->ddq_peak = fabsf(s.q_ddot[j]);
      if (s.q[j] < out->q_lo) out->q_lo = s.q[j];
      if (s.q[j] > out->q_hi) out->q_hi = s.q[j];
    }
  }
}

/* 1. 静止起步的边界条件, 以及零边界条件下的峰值闭式解。 */
static void test_boundary(void) {
  EC_TrajLimits lim;
  EC_JointStateReference start;
  EC_JointStateReference s;
  EC_TrajPlan plan;
  Scan scan;
  const float target[EC_MAX_JOINTS] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
  const float T = 1.0f;
  float applied = 0.0f;

  printf("test_boundary\n");
  limits_default(&lim);
  state_set(&start, 0.0f, 0.0f, 0.0f);
  expect(ec_traj_plan_point_to_point(&plan, &lim, &start, target, T, 1000u,
                                     0.001f, 7u, &applied) == EC_TRAJ_OK,
         "p2p returns OK");
  expect_near(applied, T, 1e-6f, "duration not stretched");
  expect(plan.seg_count == 1u, "one segment");
  expect(plan.status == (unsigned)EC_TRAJ_ACTIVE, "status ACTIVE");
  expect(plan.source_id == 7u, "source_id kept");

  ec_traj_eval_time(&plan, 0.0f, &s);
  expect_near(s.q[0], 0.0f, 1e-6f, "q(0)");
  expect_near(s.q_dot[0], 0.0f, 1e-6f, "dq(0)");
  expect_near(s.q_ddot[0], 0.0f, 1e-6f, "ddq(0)");
  ec_traj_eval_time(&plan, T, &s);
  expect_near(s.q[0], 1.0f, 1e-5f, "q(T)");
  expect_near(s.q_dot[0], 0.0f, 1e-5f, "dq(T)");
  expect_near(s.q_ddot[0], 0.0f, 1e-4f, "ddq(T)");

  /* 零边界五次曲线: |dq|max = 1.875*dq/T, |ddq|max = 10/sqrt(3)*dq/T^2。 */
  scan_plan(&plan, &scan);
  expect_near(scan.dq_peak, 1.875f, 2e-3f, "peak |dq| = 1.875");
  expect_near(scan.ddq_peak, 10.0f / sqrtf(3.0f), 5e-3f, "peak |ddq| = 5.7735");
  expect(scan.q_lo >= -1e-6f && scan.q_hi <= 1.0f + 1e-6f, "no overshoot");
}

/* 2. 上一条轨迹没走完就换命令, 拼接处 q/dq/ddq 必须连续。 */
static void test_replan_c2(void) {
  EC_TrajLimits lim;
  EC_JointStateReference start;
  EC_JointStateReference mid;
  EC_JointStateReference head;
  EC_TrajPlan a;
  EC_TrajPlan b;
  const float t1[EC_MAX_JOINTS] = {1.2f, -0.8f, 0.5f, 2.0f, -1.5f, 0.3f};
  const float t2[EC_MAX_JOINTS] = {-0.4f, 1.1f, -2.0f, 0.0f, 0.9f, -1.2f};
  unsigned j;

  printf("test_replan_c2\n");
  limits_default(&lim);
  state_set(&start, 0.0f, 0.0f, 0.0f);
  expect(ec_traj_plan_point_to_point(&a, &lim, &start, t1, 1.0f, 0u, 0.001f, 1u,
                                     NULL) == EC_TRAJ_OK,
         "plan A ok");
  /* 走到 0.37 s 处被打断。 */
  expect(ec_traj_eval_time(&a, 0.37f, &mid), "eval A mid");
  expect(ec_traj_plan_point_to_point(&b, &lim, &mid, t2, 1.0f, 370u, 0.001f, 2u,
                                     NULL) == EC_TRAJ_OK,
         "plan B ok");
  expect(ec_traj_eval_time(&b, 0.0f, &head), "eval B head");
  for (j = 0; j < N_JOINT; ++j) {
    expect_near(head.q[j], mid.q[j], 1e-6f, "q continuous across replan");
    expect_near(head.q_dot[j], mid.q_dot[j], 1e-5f, "dq continuous");
    expect_near(head.q_ddot[j], mid.q_ddot[j], 1e-3f, "ddq continuous");
  }
  /* B 的终点仍然静止。 */
  expect(ec_traj_eval_time(&b, b.total_duration, &head), "eval B tail");
  for (j = 0; j < N_JOINT; ++j) {
    expect_near(head.q[j], t2[j], 1e-4f, "B reaches target");
    expect_near(head.q_dot[j], 0.0f, 1e-4f, "B ends at rest");
  }
}

/* 3. 求值越界: t < 0 回到起点, t > total 保持在终点, 且都返回 true。 */
static void test_out_of_range(void) {
  EC_TrajLimits lim;
  EC_JointStateReference start;
  EC_JointStateReference s;
  EC_TrajPlan plan;
  const float target[EC_MAX_JOINTS] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};

  printf("test_out_of_range\n");
  limits_default(&lim);
  state_set(&start, 0.0f, 0.0f, 0.0f);
  ec_traj_plan_point_to_point(&plan, &lim, &start, target, 0.5f, 100u, 0.001f,
                              0u, NULL);
  expect(ec_traj_eval_time(&plan, -5.0f, &s), "eval before start returns true");
  expect_near(s.q[0], 0.0f, 1e-6f, "q clamped to start");
  expect_near(s.q_dot[0], 0.0f, 1e-6f, "dq at start is 0");
  expect(ec_traj_eval_time(&plan, 100.0f, &s), "eval past end returns true");
  expect_near(s.q[0], 0.5f, 1e-6f, "q holds at target");
  expect_near(s.q_dot[0], 0.0f, 1e-9f, "dq exactly 0 past end");
  expect_near(s.q_ddot[0], 0.0f, 1e-9f, "ddq exactly 0 past end");
}

/* 4. 时长太短导致越限时, 构造期必须把时长拉长而不是放行。 */
static void test_feasibility_stretch(void) {
  EC_TrajLimits lim;
  EC_JointStateReference start;
  EC_TrajPlan plan;
  Scan scan;
  const float target[EC_MAX_JOINTS] = {3.0f, 3.0f, 3.0f, 3.0f, 3.0f, 3.0f};
  float applied = 0.0f;

  printf("test_feasibility_stretch\n");
  limits_default(&lim);
  state_set(&start, 0.0f, 0.0f, 0.0f);
  /* 3 rad / 0.05 s 需要 112 rad/s 和 6928 rad/s^2, 远超 2 和 20。 */
  expect(ec_traj_plan_point_to_point(&plan, &lim, &start, target, 0.05f, 0u,
                                     0.001f, 0u, &applied) == EC_TRAJ_OK,
         "aggressive command accepted after stretch");
  expect(applied > 0.05f, "duration was stretched");
  expect_near(plan.total_duration, applied, 1e-6f, "total matches applied");
  scan_plan(&plan, &scan);
  expect(scan.dq_peak <= lim.dq_max[0] * 1.001f, "dq within limit");
  expect(scan.ddq_peak <= lim.ddq_max[0] * 1.001f, "ddq within limit");
  printf("  stretched %.3f -> %.3f s, peak dq %.3f, ddq %.3f\n", 0.05,
         (double)applied, (double)scan.dq_peak, (double)scan.ddq_peak);

  /* 拉到 max_duration 仍不可行时必须报错, 而不是悄悄放行。 */
  lim.max_duration = 0.06f;
  expect(ec_traj_plan_point_to_point(&plan, &lim, &start, target, 0.05f, 0u,
                                     0.001f, 0u, NULL) == EC_TRAJ_ERR_INFEASIBLE,
         "infeasible reported when capped");
}

/* 5. 目标点本身越限: 拉长时长救不回来, 必须直接拒绝。 */
static void test_limit_reject(void) {
  EC_TrajLimits lim;
  EC_JointStateReference start;
  EC_TrajPlan plan;
  float target[EC_MAX_JOINTS] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

  printf("test_limit_reject\n");
  limits_default(&lim);
  state_set(&start, 0.0f, 0.0f, 0.0f);
  target[3] = 4.0f; /* > q_max 3.14 */
  expect(ec_traj_plan_point_to_point(&plan, &lim, &start, target, 1.0f, 0u,
                                     0.001f, 0u, NULL) == EC_TRAJ_ERR_LIMIT,
         "target above q_max rejected");
  target[3] = -4.0f;
  expect(ec_traj_plan_point_to_point(&plan, &lim, &start, target, 1.0f, 0u,
                                     0.001f, 0u, NULL) == EC_TRAJ_ERR_LIMIT,
         "target below q_min rejected");
}

/* 6. 多节点: 段与段的拼接处 q/dq/ddq 都必须连续, 且中间不停顿。 */
static void test_multi_point(void) {
  EC_TrajLimits lim;
  EC_JointStateReference start;
  EC_TrajPlan plan;
  Scan scan;
  float nodes[3][EC_MAX_JOINTS];
  const float durations[3] = {0.6f, 0.6f, 0.6f};
  unsigned i;
  unsigned j;
  float boundary;

  printf("test_multi_point\n");
  limits_default(&lim);
  state_set(&start, 0.0f, 0.0f, 0.0f);
  for (j = 0; j < N_JOINT; ++j) {
    nodes[0][j] = 0.5f;
    nodes[1][j] = 1.0f; /* 与第 0 段同向, 中间节点不该停下 */
    nodes[2][j] = 0.2f; /* 折返 */
  }
  expect(ec_traj_plan_multi_point(&plan, &lim, &start,
                                  (const float (*)[EC_MAX_JOINTS])nodes,
                                  durations, 3u, 0u, 0.001f,
                                  3u) == EC_TRAJ_OK,
         "multi_point ok");
  expect(plan.seg_count == 3u, "three segments");

  boundary = 0.0f;
  for (i = 0; i + 1u < plan.seg_count; ++i) {
    EC_JointStateReference before;
    EC_JointStateReference after;
    boundary += plan.seg[i].duration;
    expect(ec_traj_eval_time(&plan, boundary - 1.0e-5f, &before), "eval before");
    expect(ec_traj_eval_time(&plan, boundary + 1.0e-5f, &after), "eval after");
    for (j = 0; j < N_JOINT; ++j) {
      expect_near(after.q[j], before.q[j], 1e-4f, "q continuous at node");
      expect_near(after.q_dot[j], before.q_dot[j], 1e-3f, "dq continuous");
      expect_near(after.q_ddot[j], before.q_ddot[j], 1e-1f, "ddq continuous");
    }
  }
  /* 第一个中间节点同向, 速度不应为 0; 第二个是折返点, 速度应为 0。 */
  {
    EC_JointStateReference s;
    ec_traj_eval_time(&plan, plan.seg[0].duration, &s);
    expect(fabsf(s.q_dot[0]) > 0.1f, "interior node keeps moving");
    ec_traj_eval_time(&plan, plan.seg[0].duration + plan.seg[1].duration, &s);
    expect_near(s.q_dot[0], 0.0f, 1e-3f, "reversal node stops");
    ec_traj_eval_time(&plan, plan.total_duration, &s);
    expect_near(s.q[0], 0.2f, 1e-4f, "reaches last node");
    expect_near(s.q_dot[0], 0.0f, 1e-4f, "ends at rest");
  }
  scan_plan(&plan, &scan);
  expect(scan.dq_peak <= lim.dq_max[0] * 1.001f, "multi dq within limit");
  expect(scan.ddq_peak <= lim.ddq_max[0] * 1.001f, "multi ddq within limit");
}

/* 7. 刹车: 从带速度的状态减速到停, 起点 C2 连续, 加速度不越限。 */
static void test_brake(void) {
  EC_TrajLimits lim;
  EC_JointStateReference start;
  EC_JointStateReference s;
  EC_TrajPlan plan;
  Scan scan;
  unsigned j;

  printf("test_brake\n");
  limits_default(&lim);
  state_set(&start, 0.3f, 1.5f, 0.0f);
  expect(ec_traj_plan_brake(&plan, &lim, &start, 500u, 0.001f, 9u) ==
             EC_TRAJ_OK,
         "brake ok");
  expect(plan.status == (unsigned)EC_TRAJ_BRAKING, "status BRAKING");
  expect(ec_traj_eval_time(&plan, 0.0f, &s), "eval brake head");
  for (j = 0; j < N_JOINT; ++j) {
    expect_near(s.q[j], 0.3f, 1e-6f, "brake starts at current q");
    expect_near(s.q_dot[j], 1.5f, 1e-5f, "brake starts at current dq");
  }
  expect(ec_traj_eval_time(&plan, plan.total_duration, &s), "eval brake tail");
  for (j = 0; j < N_JOINT; ++j) {
    expect_near(s.q_dot[j], 0.0f, 1e-4f, "brake ends at rest");
  }
  scan_plan(&plan, &scan);
  expect(scan.ddq_peak <= lim.ddq_max[0] * 1.001f, "brake ddq within limit");
  expect(scan.dq_peak <= lim.dq_max[0] * 1.001f, "brake dq within limit");
  printf("  brake T = %.4f s, peak ddq %.3f\n", (double)plan.total_duration,
         (double)scan.ddq_peak);

  /* 本来就停着: 零段计划, 立即视为刹车完成。 */
  state_set(&start, -0.7f, 0.0f, 0.0f);
  expect(ec_traj_plan_brake(&plan, &lim, &start, 0u, 0.001f, 9u) == EC_TRAJ_OK,
         "brake from rest ok");
  expect(plan.seg_count == 0u, "brake from rest has no segment");
  expect(ec_traj_eval_time(&plan, 0.0f, &s), "eval rest brake");
  expect_near(s.q[0], -0.7f, 1e-6f, "holds current q");
  expect_near(s.q_dot[0], 0.0f, 1e-9f, "no velocity");
}

/* 8. tick 求值, 含计数器回绕。 */
static void test_tick_wrap(void) {
  EC_TrajLimits lim;
  EC_JointStateReference start;
  EC_JointStateReference by_tick;
  EC_JointStateReference by_time;
  EC_TrajPlan plan;
  const float target[EC_MAX_JOINTS] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
  const uint32_t t0 = 0xFFFFFFF0u; /* 16 个 tick 之后回绕 */

  printf("test_tick_wrap\n");
  limits_default(&lim);
  state_set(&start, 0.0f, 0.0f, 0.0f);
  ec_traj_plan_point_to_point(&plan, &lim, &start, target, 1.0f, t0, 0.001f, 0u,
                              NULL);
  /* t0 + 300 tick 已经越过 0, 相对时间必须仍是 0.300 s。 */
  expect(ec_traj_eval_tick(&plan, t0 + 300u, &by_tick), "eval by wrapped tick");
  expect(ec_traj_eval_time(&plan, 0.300f, &by_time), "eval by time");
  expect_near(by_tick.q[0], by_time.q[0], 1e-6f, "wrapped tick matches time");
  expect_near(by_tick.q_dot[0], by_time.q_dot[0], 1e-5f, "wrapped dq matches");
  /* 落在起点之前也不能被误判成"很久以后"。 */
  expect(ec_traj_eval_tick(&plan, t0 - 5u, &by_tick), "eval before t0");
  expect_near(by_tick.q[0], 0.0f, 1e-6f, "before t0 clamps to start");
}

/* 9. 双缓冲发布: 读者在 commit 前后各自看到完整的一份计划。 */
static void test_channel(void) {
  static EC_TrajChannel ch;
  EC_TrajLimits lim;
  EC_JointStateReference start;
  EC_JointStateReference s;
  const float q0[EC_MAX_JOINTS] = {0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f};
  const float target[EC_MAX_JOINTS] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
  const EC_TrajPlan *read_before;
  const EC_TrajPlan *read_after;
  EC_TrajPlan *write_slot;

  printf("test_channel\n");
  limits_default(&lim);
  expect(ec_traj_channel_init(&ch, N_JOINT, q0, 0u, 0.001f), "channel init");
  read_before = ec_traj_channel_read(&ch);
  expect(read_before != NULL, "read gives a plan");
  expect(read_before->status == (unsigned)EC_TRAJ_HOLD, "init status HOLD");
  expect(read_before->seg_count == 0u, "init plan is pure hold");
  expect(ec_traj_eval_time(read_before, 123.0f, &s), "init plan evaluable");
  expect_near(s.q[0], 0.1f, 1e-6f, "init plan holds q0");

  write_slot = ec_traj_channel_begin_write(&ch);
  expect(write_slot != read_before, "write slot differs from read slot");
  state_set(&start, 0.1f, 0.0f, 0.0f);
  expect(ec_traj_plan_point_to_point(write_slot, &lim, &start, target, 1.0f, 0u,
                                     0.001f, 4u, NULL) == EC_TRAJ_OK,
         "build into write slot");
  /* 还没 commit, 读者必须仍看到旧计划。 */
  expect(ec_traj_channel_read(&ch) == read_before, "read unchanged before commit");
  expect(ec_traj_channel_read(&ch)->status == (unsigned)EC_TRAJ_HOLD,
         "old plan still HOLD");

  ec_traj_channel_commit(&ch);
  read_after = ec_traj_channel_read(&ch);
  expect(read_after == write_slot, "read follows commit");
  expect(read_after->status == (unsigned)EC_TRAJ_ACTIVE, "new plan ACTIVE");
  expect(read_after->seq > read_before->seq, "seq increased");
  expect(read_after->source_id == 4u, "source_id published");
  /* 再 commit 一次应该翻回原来那个槽。 */
  expect(ec_traj_channel_begin_write(&ch) == read_before, "slots alternate");
}

/* 10. 增量式遥控: 固定平滑时长, 每帧都从当前参考状态重新起一段。 */
static void test_fixed_duration_jog(void) {
  EC_TrajLimits lim;
  EC_JointStateReference cur;
  EC_TrajPlan plan;
  float target[EC_MAX_JOINTS];
  const float smooth_T = 0.1f;
  unsigned frame;
  unsigned j;
  float max_ddq = 0.0f;
  float max_dq = 0.0f;

  printf("test_fixed_duration_jog\n");
  limits_default(&lim);
  state_set(&cur, 0.0f, 0.0f, 0.0f);
  for (j = 0; j < N_JOINT; ++j) target[j] = 0.0f;

  /* 模拟 40 帧、帧间隔在 14 ms 与 50 ms 之间抖动的增量流。每帧把增量累加
   * 到目标上, 然后从"此刻的参考状态"重新拟合一段固定 100 ms 的曲线。
   * 关键性质: 采样时刻永远落在上一段内部, 所以起点带着非零 dq/ddq,
   * 拼接必须连续, 而且整段流程不能越限。 */
  for (frame = 0; frame < 40u; ++frame) {
    const float dt = (frame % 2u) ? 0.014f : 0.050f;
    EC_JointStateReference next;
    for (j = 0; j < N_JOINT; ++j) target[j] += 0.02f;
    expect(ec_traj_plan_point_to_point(&plan, &lim, &cur, target, smooth_T, 0u,
                                       0.001f, 5u, NULL) == EC_TRAJ_OK,
           "jog frame accepted");
    /* 在这一帧内按 2 ms 控制周期走, 检查过程量。 */
    {
      float t;
      for (t = 0.0f; t < dt; t += 0.002f) {
        EC_JointStateReference s;
        expect(ec_traj_eval_time(&plan, t, &s), "jog eval");
        for (j = 0; j < N_JOINT; ++j) {
          if (fabsf(s.q_dot[j]) > max_dq) max_dq = fabsf(s.q_dot[j]);
          if (fabsf(s.q_ddot[j]) > max_ddq) max_ddq = fabsf(s.q_ddot[j]);
        }
      }
    }
    expect(ec_traj_eval_time(&plan, dt, &next), "jog eval at frame end");
    cur = next;
  }
  /* 跟随滞后大约一个平滑时长, 所以位置应该接近但略落后于目标。 */
  expect(cur.q[0] < target[0], "reference lags target by design");
  expect(cur.q[0] > target[0] - 0.4f, "lag stays bounded");
  expect(max_dq <= lim.dq_max[0] * 1.02f, "jog dq within limit");
  expect(max_ddq <= lim.ddq_max[0] * 1.02f, "jog ddq within limit");
  printf("  after 40 frames: target %.3f, ref %.3f, lag %.3f rad\n",
         (double)target[0], (double)cur.q[0], (double)(target[0] - cur.q[0]));
  printf("  peak dq %.3f, peak ddq %.3f\n", (double)max_dq, (double)max_ddq);
}

int main(void) {
  test_boundary();
  test_replan_c2();
  test_out_of_range();
  test_feasibility_stretch();
  test_limit_reject();
  test_multi_point();
  test_brake();
  test_tick_wrap();
  test_channel();
  test_fixed_duration_jog();
  printf("\n%d checks, %d failures -> %s\n", g_checks, g_fail,
         g_fail ? "FAIL" : "PASS");
  return g_fail ? 1 : 0;
}
