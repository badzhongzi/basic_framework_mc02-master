#include "ec_traj.h"

#include <math.h>
#include <string.h>

/*
 * 屏障只需要阻止编译器重排。Cortex-M 是单核, 对普通内存的写入对同一个核
 * 上的其他执行流天然有序, 所以不需要真正的 DMB 指令; 用编译器内建的
 * seq_cst fence 既拿到了这个效果, 又不引入对 CMSIS 头文件的依赖。
 */
#if defined(__GNUC__)
#define EC_TRAJ_BARRIER() __atomic_thread_fence(__ATOMIC_SEQ_CST)
#else
#define EC_TRAJ_BARRIER() do { } while (0)
#endif

/* 段时长的绝对下限, 防止除零。 */
#define EC_TRAJ_MIN_DURATION 1.0e-4f

static float traj_clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

/*
 * 五次 Hermite 拟合, 单关节, 归一化时间。
 *
 * 边界条件 q(0)=q0, q'(0)=V0, q''(0)=A0, q(1)=q1, q'(1)=V1, q''(1)=A1,
 * 其中 V = v*T, A = a*T^2 是把物理导数换算到归一化时间的结果。
 * 低三个系数直接由起点条件给出, 高三个系数解 3x3 方程组:
 *   [ 1  1  1] [c3]   [d0]
 *   [ 3  4  5] [c4] = [d1]
 *   [ 6 12 20] [c5]   [d2]
 * 该矩阵的逆是 [[10,-4,0.5],[-15,7,-1],[6,-3,0.5]]。
 */
static void traj_fit_joint(float *c, float q0, float v0, float a0, float q1,
                           float v1, float a1, float duration) {
  const float t2 = duration * duration;
  const float bv0 = v0 * duration;
  const float ba0 = a0 * t2;
  const float bv1 = v1 * duration;
  const float ba1 = a1 * t2;
  const float d0 = (q1 - q0) - bv0 - 0.5f * ba0;
  const float d1 = bv1 - bv0 - ba0;
  const float d2 = ba1 - ba0;

  c[0] = q0;
  c[1] = bv0;
  c[2] = 0.5f * ba0;
  c[3] = 10.0f * d0 - 4.0f * d1 + 0.5f * d2;
  c[4] = -15.0f * d0 + 7.0f * d1 - d2;
  c[5] = 6.0f * d0 - 3.0f * d1 + 0.5f * d2;
}

/* 在归一化时刻 s 求单关节的 q / dq / ddq; 出参允许为 NULL。 */
static void traj_seg_eval_joint(const EC_TrajSeg *seg, uint8_t joint, float s,
                                float *q, float *dq, float *ddq) {
  const float *c = seg->c[joint];
  const float inv = 1.0f / seg->duration;

  if (q) {
    *q = c[0] + s * (c[1] + s * (c[2] + s * (c[3] + s * (c[4] + s * c[5]))));
  }
  if (dq) {
    *dq = (c[1] + s * (2.0f * c[2] + s * (3.0f * c[3] +
           s * (4.0f * c[4] + s * (5.0f * c[5]))))) * inv;
  }
  if (ddq) {
    *ddq = (2.0f * c[2] + s * (6.0f * c[3] + s * (12.0f * c[4] +
            s * (20.0f * c[5])))) * inv * inv;
  }
}

/* 一段的可行性检查结果。ratio > 1 表示越限, 数值即需要的缩放倍率。 */
typedef struct {
  float v_ratio;
  float a_ratio;
  bool pos_violation;
} TrajCheck;

/*
 * 在段上均匀采样检查速度、加速度和位置。
 *
 * 五次多项式的导数极值点要解四次/三次方程, 在 MCU 上不划算; 而这个检查
 * 每条命令只做一次 (不是每个控制周期), 所以直接密集采样更省事也更稳。
 * 采样会略微低估真实峰值, 因此对速度和加速度留 2% 余量。
 */
static void traj_check_seg(const EC_TrajSeg *seg, const EC_TrajLimits *limits,
                           uint8_t joint_count, bool check_position,
                           TrajCheck *out) {
  const uint8_t samples = (uint8_t)EC_TRAJ_CHECK_SAMPLES;
  uint8_t k;
  uint8_t j;

  out->v_ratio = 0.0f;
  out->a_ratio = 0.0f;
  out->pos_violation = false;

  for (k = 0; k < samples; ++k) {
    const float s = (float)k / (float)(samples - 1u);
    for (j = 0; j < joint_count; ++j) {
      float q;
      float dq;
      float ddq;
      traj_seg_eval_joint(seg, j, s, &q, &dq, &ddq);
      if (limits->dq_max[j] > 0.0f) {
        const float r = fabsf(dq) / limits->dq_max[j];
        if (r > out->v_ratio) out->v_ratio = r;
      }
      if (limits->ddq_max[j] > 0.0f) {
        const float r = fabsf(ddq) / limits->ddq_max[j];
        if (r > out->a_ratio) out->a_ratio = r;
      }
      if (check_position &&
          (q < limits->q_min[j] || q > limits->q_max[j])) {
        out->pos_violation = true;
      }
    }
  }
  out->v_ratio *= 1.02f;
  out->a_ratio *= 1.02f;
}

/*
 * 拟合一段并在必要时拉长时长直到满足约束。
 *
 * 缩放倍率的选取: 在零边界条件下速度峰值正比于 1/T、加速度峰值正比于
 * 1/T^2, 所以想把越限倍率 rv、ra 压回 1, 需要的倍率分别是 rv 和 sqrt(ra);
 * 取两者较大值即可一步到位。起点带非零 dq/ddq 时这个关系不再严格成立
 * (归一化边界值 V0 = v0*T 本身随 T 变), 所以外面套一个迭代重试。
 *
 * 位置越限只发生在起点带速度、多项式冲过终点的情况; 拉长时长能给出更多
 * 减速距离从而减小超调, 但关系不解析, 所以对这一项用固定 1.5 倍试探。
 */
static EC_TrajResult traj_build_checked(
    EC_TrajSeg *seg, const EC_TrajLimits *limits, const float *q0,
    const float *v0, const float *a0, const float *q1, const float *v1,
    const float *a1, float duration, uint8_t joint_count,
    bool check_position, float *applied_duration) {
  const float min_t = (limits->min_duration > EC_TRAJ_MIN_DURATION)
                          ? limits->min_duration
                          : EC_TRAJ_MIN_DURATION;
  const float max_t = (limits->max_duration > min_t) ? limits->max_duration
                                                     : min_t;
  float t = traj_clampf(duration, min_t, max_t);
  uint8_t iter;

  for (iter = 0; iter <= (uint8_t)EC_TRAJ_STRETCH_ITERS; ++iter) {
    TrajCheck check;
    uint8_t j;
    float scale;

    seg->duration = t;
    for (j = 0; j < joint_count; ++j) {
      traj_fit_joint(seg->c[j], q0[j], v0 ? v0[j] : 0.0f, a0 ? a0[j] : 0.0f,
                     q1[j], v1 ? v1[j] : 0.0f, a1 ? a1[j] : 0.0f, t);
    }
    /* 未使用的关节位清零, 便于整份计划按固定大小复制和比较。 */
    for (j = joint_count; j < (uint8_t)EC_MAX_JOINTS; ++j) {
      memset(seg->c[j], 0, sizeof(seg->c[j]));
    }

    traj_check_seg(seg, limits, joint_count, check_position, &check);
    if (check.v_ratio <= 1.0f && check.a_ratio <= 1.0f &&
        !check.pos_violation) {
      if (applied_duration) *applied_duration = t;
      return EC_TRAJ_OK;
    }
    if (t >= max_t) break; /* 已经拉到上限且仍不可行。 */

    scale = check.v_ratio;
    if (check.a_ratio > 1.0f) {
      const float sa = sqrtf(check.a_ratio);
      if (sa > scale) scale = sa;
    }
    if (check.pos_violation && scale < 1.5f) scale = 1.5f;
    if (scale < 1.05f) scale = 1.05f;
    t = traj_clampf(t * scale, min_t, max_t);
  }
  return EC_TRAJ_ERR_INFEASIBLE;
}

/* ------------------------------- 求值 ------------------------------- */

/*
 * 定位 t 落在哪一段, 并给出该段内的归一化时间。
 * 返回 NULL 表示"没有段可用": 纯保持计划, 或 t 已越过最后一段的末尾。
 */
static const EC_TrajSeg *traj_locate(const EC_TrajPlan *plan, float t,
                                     float *s_out) {
  uint8_t i;

  if (plan->seg_count == 0u) return NULL;
  if (!(t > 0.0f)) { /* 含 t < 0 和 NaN */
    *s_out = 0.0f;
    return &plan->seg[0];
  }
  for (i = 0; i < plan->seg_count; ++i) {
    const EC_TrajSeg *seg = &plan->seg[i];
    if (t < seg->duration) {
      *s_out = t / seg->duration;
      return seg;
    }
    t -= seg->duration;
  }
  return NULL;
}

static bool traj_plan_valid(const EC_TrajPlan *plan) {
  return plan && plan->joint_count > 0u &&
         plan->joint_count <= (uint8_t)EC_MAX_JOINTS &&
         plan->seg_count <= (uint8_t)EC_TRAJ_MAX_SEG;
}

bool ec_traj_eval_time(const EC_TrajPlan *plan, float t,
                       EC_JointStateReference *out) {
  const EC_TrajSeg *seg;
  float s = 0.0f;
  uint8_t j;

  if (!traj_plan_valid(plan) || !out) return false;
  out->joint_count = plan->joint_count;

  seg = traj_locate(plan, t, &s);
  if (!seg) {
    for (j = 0; j < plan->joint_count; ++j) {
      out->q[j] = plan->q_hold[j];
      out->q_dot[j] = 0.0f;
      out->q_ddot[j] = 0.0f;
    }
    return true;
  }
  for (j = 0; j < plan->joint_count; ++j) {
    traj_seg_eval_joint(seg, j, s, &out->q[j], &out->q_dot[j], &out->q_ddot[j]);
  }
  return true;
}

bool ec_traj_eval_tick(const EC_TrajPlan *plan, uint32_t tick,
                       EC_JointStateReference *out) {
  int32_t delta;

  if (!traj_plan_valid(plan)) return false;
  /* 无符号相减再转有符号, tick 计数器回绕时依然给出正确的相对量。 */
  delta = (int32_t)(tick - plan->t0_tick);
  return ec_traj_eval_time(plan, (float)delta * plan->tick_period, out);
}

bool ec_traj_eval_joint_time(const EC_TrajPlan *plan, float t, uint8_t joint,
                             float *q, float *dq, float *ddq) {
  const EC_TrajSeg *seg;
  float s = 0.0f;

  if (!traj_plan_valid(plan) || joint >= plan->joint_count) return false;

  seg = traj_locate(plan, t, &s);
  if (!seg) {
    if (q) *q = plan->q_hold[joint];
    if (dq) *dq = 0.0f;
    if (ddq) *ddq = 0.0f;
    return true;
  }
  traj_seg_eval_joint(seg, joint, s, q, dq, ddq);
  return true;
}

/* ------------------------------- 构造 ------------------------------- */

bool ec_traj_limits_init(EC_TrajLimits *limits, uint8_t joint_count) {
  uint8_t j;

  if (!limits || joint_count == 0u || joint_count > (uint8_t)EC_MAX_JOINTS) {
    return false;
  }
  memset(limits, 0, sizeof(*limits));
  limits->joint_count = joint_count;
  for (j = 0; j < joint_count; ++j) {
    limits->q_min[j] = -EC_PI;
    limits->q_max[j] = EC_PI;
    limits->dq_max[j] = 1.0f;
    limits->ddq_max[j] = 10.0f;
  }
  limits->min_duration = 0.02f;
  limits->max_duration = 5.0f;
  return true;
}

static bool traj_args_ok(const EC_TrajLimits *limits,
                         const EC_JointStateReference *start) {
  return limits && start && limits->joint_count > 0u &&
         limits->joint_count <= (uint8_t)EC_MAX_JOINTS &&
         start->joint_count >= limits->joint_count;
}

static void traj_plan_begin(EC_TrajPlan *plan, uint8_t joint_count,
                            uint32_t t0_tick, float tick_period,
                            uint8_t status, uint8_t source_id) {
  memset(plan, 0, sizeof(*plan));
  plan->t0_tick = t0_tick;
  plan->tick_period = (tick_period > 0.0f) ? tick_period : 0.001f;
  plan->joint_count = joint_count;
  plan->status = status;
  plan->source_id = source_id;
}

bool ec_traj_plan_hold(EC_TrajPlan *plan, uint8_t joint_count,
                       const float *q_hold, uint32_t t0_tick,
                       float tick_period, uint8_t status, uint8_t source_id) {
  uint8_t j;

  if (!plan || !q_hold || joint_count == 0u ||
      joint_count > (uint8_t)EC_MAX_JOINTS) {
    return false;
  }
  traj_plan_begin(plan, joint_count, t0_tick, tick_period, status, source_id);
  for (j = 0; j < joint_count; ++j) plan->q_hold[j] = q_hold[j];
  /* seg_count 为 0, total_duration 为 0: 任何时刻求值都落到保持分支。 */
  return true;
}

EC_TrajResult ec_traj_plan_point_to_point(
    EC_TrajPlan *plan, const EC_TrajLimits *limits,
    const EC_JointStateReference *start, const float *q_target, float duration,
    uint32_t t0_tick, float tick_period, uint8_t source_id,
    float *applied_duration) {
  uint8_t n;
  uint8_t j;
  float applied = 0.0f;
  EC_TrajResult rc;

  if (!plan || !q_target || !traj_args_ok(limits, start)) {
    return EC_TRAJ_ERR_ARG;
  }
  n = limits->joint_count;
  for (j = 0; j < n; ++j) {
    if (q_target[j] < limits->q_min[j] || q_target[j] > limits->q_max[j]) {
      return EC_TRAJ_ERR_LIMIT;
    }
  }

  traj_plan_begin(plan, n, t0_tick, tick_period, (uint8_t)EC_TRAJ_ACTIVE,
                  source_id);
  rc = traj_build_checked(&plan->seg[0], limits, start->q, start->q_dot,
                          start->q_ddot, q_target, NULL, NULL, duration, n,
                          true, &applied);
  if (rc != EC_TRAJ_OK) return rc;

  plan->seg_count = 1u;
  plan->total_duration = applied;
  for (j = 0; j < n; ++j) plan->q_hold[j] = q_target[j];
  if (applied_duration) *applied_duration = applied;
  return EC_TRAJ_OK;
}

EC_TrajResult ec_traj_plan_multi_point(
    EC_TrajPlan *plan, const EC_TrajLimits *limits,
    const EC_JointStateReference *start, const float nodes[][EC_MAX_JOINTS],
    const float *durations, uint8_t node_count, uint32_t t0_tick,
    float tick_period, uint8_t source_id) {
  /* 两个速度缓冲乒乓使用, 避免开 [EC_TRAJ_MAX_SEG][EC_MAX_JOINTS] 的大数组:
   * 拟合第 i 段只需要节点 i-1 和节点 i 的速度。 */
  float vbuf[2][EC_MAX_JOINTS];
  const float *q_prev;
  const float *v_prev;
  const float *a_prev;
  uint8_t cur = 0u;
  uint8_t n;
  uint8_t i;
  uint8_t j;
  float total = 0.0f;

  if (!plan || !nodes || !durations || !traj_args_ok(limits, start)) {
    return EC_TRAJ_ERR_ARG;
  }
  if (node_count == 0u) return EC_TRAJ_ERR_ARG;
  if (node_count > (uint8_t)EC_TRAJ_MAX_SEG) return EC_TRAJ_ERR_SEG;
  n = limits->joint_count;
  for (i = 0; i < node_count; ++i) {
    if (!(durations[i] > 0.0f)) return EC_TRAJ_ERR_ARG;
    for (j = 0; j < n; ++j) {
      if (nodes[i][j] < limits->q_min[j] || nodes[i][j] > limits->q_max[j]) {
        return EC_TRAJ_ERR_LIMIT;
      }
    }
  }

  traj_plan_begin(plan, n, t0_tick, tick_period, (uint8_t)EC_TRAJ_ACTIVE,
                  source_id);
  q_prev = start->q;
  v_prev = start->q_dot;
  a_prev = start->q_ddot;

  for (i = 0; i < node_count; ++i) {
    float *v_end = vbuf[cur];
    float applied = 0.0f;
    EC_TrajResult rc;

    if (i + 1u == node_count) {
      memset(v_end, 0, sizeof(vbuf[cur]));
    } else {
      /* 中间节点速度用无量纲中心差分: 前后两段方向一致时取平均速度,
       * 方向相反 (该节点是局部极值) 时取 0, 否则会冲过节点再折回。 */
      for (j = 0; j < n; ++j) {
        const float s1 = (nodes[i][j] - q_prev[j]) / durations[i];
        const float s2 = (nodes[i + 1u][j] - nodes[i][j]) / durations[i + 1u];
        v_end[j] = (s1 * s2 > 0.0f) ? (0.5f * (s1 + s2)) : 0.0f;
      }
    }
    /* 中间节点加速度两侧都取 0, 所以拼接处 q、dq、ddq 全部连续。
     * a_prev 传 NULL 即表示零加速度起点。 */
    rc = traj_build_checked(&plan->seg[i], limits, q_prev, v_prev, a_prev,
                            nodes[i], v_end, NULL, durations[i], n, true,
                            &applied);
    if (rc != EC_TRAJ_OK) return rc;
    total += applied;

    q_prev = nodes[i];
    v_prev = v_end;
    a_prev = NULL;
    cur ^= 1u;
  }

  plan->seg_count = node_count;
  plan->total_duration = total;
  for (j = 0; j < n; ++j) plan->q_hold[j] = nodes[node_count - 1u][j];
  return EC_TRAJ_OK;
}

EC_TrajResult ec_traj_plan_brake(EC_TrajPlan *plan,
                                 const EC_TrajLimits *limits,
                                 const EC_JointStateReference *start,
                                 uint32_t t0_tick, float tick_period,
                                 uint8_t source_id) {
  float q_stop[EC_MAX_JOINTS];
  float applied = 0.0f;
  float t0;
  bool moving = false;
  uint8_t n;
  uint8_t j;
  EC_TrajResult rc;

  if (!plan || !traj_args_ok(limits, start)) return EC_TRAJ_ERR_ARG;
  n = limits->joint_count;
  t0 = limits->min_duration;

  /*
   * 刹车时长初值。对于 (q0, v0, 0) -> (q0 + v0*T/2, 0, 0) 的五次曲线,
   * 归一化二阶导恰好是 6*v0*T*s*(s-1), 峰值在 s=0.5 处为 1.5*|v0|/T,
   * 而一阶导 v0*(1-3s^2+2s^3) 在 [0,1] 上单调降到 0、不会超过 |v0|。
   * 所以只有加速度是约束项, T = 1.6*|v0|/a_max 留出一点余量即可;
   * 起点加速度非零的情形由 traj_build_checked 的拉伸循环兜住。
   */
  for (j = 0; j < n; ++j) {
    if (fabsf(start->q_dot[j]) > 1.0e-4f ||
        fabsf(start->q_ddot[j]) > 1.0e-3f) {
      moving = true;
    }
    if (limits->ddq_max[j] > 0.0f) {
      const float need = 1.6f * fabsf(start->q_dot[j]) / limits->ddq_max[j];
      if (need > t0) t0 = need;
    }
  }
  if (!moving) {
    /* 本来就停着。给一份零段的 BRAKING 计划, 监督层下一拍就能判定刹车
     * 已完成并转入 HOLD, 不用为这种情况单开一条分支。 */
    return ec_traj_plan_hold(plan, n, start->q, t0_tick, tick_period,
                             (uint8_t)EC_TRAJ_BRAKING, source_id)
               ? EC_TRAJ_OK
               : EC_TRAJ_ERR_ARG;
  }

  for (j = 0; j < n; ++j) {
    q_stop[j] = traj_clampf(start->q[j] + 0.5f * start->q_dot[j] * t0,
                            limits->q_min[j], limits->q_max[j]);
  }
  traj_plan_begin(plan, n, t0_tick, tick_period, (uint8_t)EC_TRAJ_BRAKING,
                  source_id);
  /*
   * 刹车不做位置门禁。目标点已经夹进限位, 但带着速度进来时多项式难免有
   * 一点超调; 为了消掉超调去拉长时长只会让它滑得更远, 硬截断又会在参考
   * 值上造成跳变。这里选择如实生成可跟随的减速曲线, 是否越界交给监督层
   * 判定和上报。
   */
  rc = traj_build_checked(&plan->seg[0], limits, start->q, start->q_dot,
                          start->q_ddot, q_stop, NULL, NULL, t0, n, false,
                          &applied);
  if (rc != EC_TRAJ_OK) return rc;

  plan->seg_count = 1u;
  plan->total_duration = applied;
  for (j = 0; j < n; ++j) plan->q_hold[j] = q_stop[j];
  return EC_TRAJ_OK;
}

/* ------------------------------- 发布 ------------------------------- */

bool ec_traj_channel_init(EC_TrajChannel *channel, uint8_t joint_count,
                          const float *q_hold, uint32_t t0_tick,
                          float tick_period) {
  if (!channel) return false;
  memset(channel, 0, sizeof(*channel));
  if (!ec_traj_plan_hold(&channel->slot[0], joint_count, q_hold, t0_tick,
                         tick_period, (uint8_t)EC_TRAJ_HOLD, 0u)) {
    return false;
  }
  channel->slot[1] = channel->slot[0];
  channel->active = 0u;
  channel->next_seq = 1u;
  return true;
}

EC_TrajPlan *ec_traj_channel_begin_write(EC_TrajChannel *channel) {
  if (!channel) return NULL;
  return &channel->slot[channel->active ^ 1u];
}

void ec_traj_channel_commit(EC_TrajChannel *channel) {
  uint32_t idx;

  if (!channel) return;
  idx = channel->active ^ 1u;
  channel->slot[idx].seq = channel->next_seq++;
  EC_TRAJ_BARRIER();
  channel->active = idx;
  EC_TRAJ_BARRIER();
}

const EC_TrajPlan *ec_traj_channel_read(const EC_TrajChannel *channel) {
  if (!channel) return NULL;
  return &channel->slot[channel->active];
}
