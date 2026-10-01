#include "ins_arm.h"

#include <string.h>

/*
 * ikine5 使用的几何参数，单位 m。
 *
 * 注意: 这三个长度仍是真实机械臂的尺寸, 与下面已换成 openarm.urdf 的 MDH
 * 模型不一致. 逆解当前解不出来、笛卡尔规划也未启用, 所以先不动; 等逆解
 * 可用时, 这里要么跟着模型一起换, 要么直接从 MDH 表推导出来.
 */
static const float kIkine5Length[3] = {0.335f, 0.280f, 0.074f};

/*
 * 把固定尺寸的 ec_ikine5 输出适配到 EC_Arm 的通用回调布局。
 * 回调中的 solution 是 [EC_MAX_IK_SOLUTIONS][EC_MAX_JOINTS]，而
 * ec_ikine5 的输出固定为 [8][6]，因此这里必须逐元素复制。
 */
static uint8_t ins_arm_solve_ikine(
    EC_ModuleCartesianCmd command, const float *free_joints,
    uint8_t joint_count,
    float solution[EC_MAX_IK_SOLUTIONS][EC_MAX_JOINTS], void *user) {
  float raw[8][6];
  uint8_t count;
  uint8_t i;
  uint8_t j;

  (void)free_joints;
  (void)user;
  if (!solution || joint_count != INS_ARM_6DOF_COUNT) return 0;

  count = ec_ikine5(command, kIkine5Length, raw);
  if (count > EC_MAX_IK_SOLUTIONS) count = EC_MAX_IK_SOLUTIONS;
  for (i = 0; i < count; ++i) {
    for (j = 0; j < INS_ARM_6DOF_COUNT; ++j) {
      solution[i][j] = raw[i][j];
    }
  }
  return count;
}

bool ins_arm_init(InsArm *instance) {
  /*
   * 当前这套参数来自占位模型 openarm.urdf (与 MPC 生成脚本同一个模型),
   * 不是真实机械臂. 真实 MDH 与动力学参数标定完成后整体替换本段.
   * 换参数时 MPC 侧的 mpc_config.py 必须同步, 否则 Ad/Bd、LQR 增益和
   * DTAU_MAX 会与在线模型脱节.
   *
   * URDF 本体: 6 个 revolute 关节沿 base z 轴每 0.1 m 一个, 无姿态偏置,
   * 关节轴序 z, y, y, z, y, z; 每根连杆 mass 1.0 kg, 质心在自身 link 系
   * (0, 0, 0.05), 惯量 0.01 对角; 限位 +-3.14 rad.
   *
   * 由 URDF 反推到 Craig (modified DH) 的过程与逐点校验见
   * tests/arm_model/verify_mdh.py: 501 组随机构型下末端位置最大偏差
   * 2e-16 m, 且 MDH 第 6 系与 URDF link6 系完全重合 (工具旋转为单位阵).
   *
   * 参数顺序: offset(rad), d(m), a(m), alpha(rad); 其中 d 为 d_i,
   * offset 为 theta_i 偏置, a/alpha 实为 a_{i-1}/alpha_{i-1}.
   * d 与 a 之和 0.6 m, 与 URDF 链总长一致.
   *
   * 真实机械臂原表 (标定完成后恢复):
   *   {0, 0, 0, 0} {0, 0, 0, +pi/2} {+pi/2, 0, 0.335, 0}
   *   {0, 0.280, 0, +pi/2} {pi, 0, 0, +pi/2} {0, 0.074, 0, +pi/2}
   */
  static const float mdh[INS_ARM_6DOF_COUNT][4] = {
      {0.0f, 0.2f, 0.0f, 0.0f},
      {-EC_PI / 2.0f, 0.0f, 0.0f, -EC_PI / 2.0f},
      {EC_PI / 2.0f, 0.0f, 0.1f, 0.0f},
      {0.0f, 0.2f, 0.0f, EC_PI / 2.0f},
      {0.0f, 0.0f, 0.0f, -EC_PI / 2.0f},
      {0.0f, 0.1f, 0.0f, EC_PI / 2.0f},
  };
  /*
   * URDF 六个关节限位都是 +-3.14, 不是真实机械臂的可达范围.
   * 真实机械臂原限位 (标定完成后恢复), 单位 deg:
   *   qmin {-170, +30, -165, -180, -120, -100}
   *   qmax {+170, +165, -60, +180, +120, +100}
   */
  static const float qmin[INS_ARM_6DOF_COUNT] = {
      -3.14f, -3.14f, -3.14f, -3.14f, -3.14f, -3.14f,
  };
  static const float qmax[INS_ARM_6DOF_COUNT] = {
      3.14f, 3.14f, 3.14f, 3.14f, 3.14f, 3.14f,
  };
  /* URDF 每根连杆 1.0 kg. 真实机械臂原值 {0, 0.535, 1.776, 0, 0.647, 0}. */
  static const float mass[INS_ARM_6DOF_COUNT] = {
      1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
  };
  /*
   * 质心表达在 MDH 连杆系 (ec_serial_link_inverse_dynamics 中按
   * body_origin[i] + R_i * centroid 使用). URDF 的 (0, 0, 0.05) 经
   * verify_mdh.py 转换到各 MDH 连杆系后如下.
   *
   * 真实机械臂原值:
   *   {0,0,0} {0.198405,0,0} {0,-0.108908,0} {0,0,0} {0,-0.041179,0} {0,0,0}
   */
  static const EC_Vec3 centroid[INS_ARM_6DOF_COUNT] = {
      {{0.0f, 0.0f, -0.05f}},
      {{0.05f, 0.0f, 0.0f}},
      {{0.0f, -0.05f, 0.0f}},
      {{0.0f, 0.0f, -0.05f}},
      {{0.0f, -0.05f, 0.0f}},
      {{0.0f, 0.0f, 0.05f}},
  };
  /*
   * 绕质心的转动惯量, 表达在 MDH 连杆系. URDF 给的是 0.01 对角各向同性张量,
   * 各向同性张量满足 R * (0.01*I) * R^T = 0.01*I, 与坐标系选取无关, 所以
   * 从 URDF link 系搬到 MDH 连杆系不需要任何变换.
   * 原表全为 0 (只给质量和质心), 现在补上, 否则前馈缺少纯惯量项.
   */
  static const EC_Mat3 inertia[INS_ARM_6DOF_COUNT] = {
      {{0.01f, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f, 0.01f}},
      {{0.01f, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f, 0.01f}},
      {{0.01f, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f, 0.01f}},
      {{0.01f, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f, 0.01f}},
      {{0.01f, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f, 0.01f}},
      {{0.01f, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f, 0.01f}},
  };

  if (!instance) return false;
  memset(instance, 0, sizeof(*instance));
  ec_serial_link_init(&instance->serial);

  for (uint8_t i = 0; i < INS_ARM_6DOF_COUNT; ++i) {
    EC_MDH mdh_param;
    EC_Link link;
    ec_mdh_init(&mdh_param, mdh[i][0], mdh[i][1], mdh[i][2], mdh[i][3],
                qmin[i], qmax[i]);
    ec_link_init(&link, &mdh_param);
    ec_link_set_dynamic(&link, mass[i], centroid[i], inertia[i]);
    if (!ec_serial_link_append(&instance->serial, &link)) return false;
  }

  if (!ec_arm_init(&instance->arm, INS_ARM_6DOF_COUNT, &instance->serial,
                   ins_arm_solve_ikine, NULL)) {
    return false;
  }
  /* 原值写死 0.005f, 而 GimbalTask 实际每 2 ms 被调用一次, 参见
   * INS_ARM_CONTROL_PERIOD 处的说明. */
  return ec_arm_set_control_period(&instance->arm, INS_ARM_CONTROL_PERIOD);
}

void ins_arm_set_feedback(InsArm *instance,
                          const float joint[INS_ARM_6DOF_COUNT]) {
  if (!instance || !joint) return;
  ec_arm_set_feedback(&instance->arm, joint);
}

void ins_arm_update(InsArm *instance) {
  if (instance) ec_arm_update(&instance->arm);
}

EC_Arm *ins_arm_state(InsArm *instance) {
  return instance ? &instance->arm : NULL;
}

const EC_Arm *ins_arm_const_state(const InsArm *instance) {
  return instance ? &instance->arm : NULL;
}

bool ins_arm_fkine(const InsArm *instance,
                   const float joint[INS_ARM_6DOF_COUNT],
                   EC_CartesianCmd *cartesian) {
  EC_SerialLink *serial;
  if (!instance || !joint || !cartesian) return false;
  /* fkine 接口不是 const，因此这里只读模型内容并暂时更新其缓存。 */
  serial = (EC_SerialLink *)&instance->serial;
  if (!ec_serial_link_fkine(serial, joint)) return false;
  cartesian->pos = ec_serial_link_get_p(serial);
  cartesian->quat = ec_serial_link_get_quat(serial);
  return true;
}

bool ins_arm_motion_finished(const InsArm *instance) {
  const EC_Arm *arm;
  if (!instance) return false;
  arm = &instance->arm;
  if (arm->planner_active) return false;
  if (arm->command.plan_type == EC_PLAN_JOINT) {
    return arm->planner_segment_index >= arm->command.node_count;
  }
  if (arm->command.plan_type == EC_PLAN_LINEAR ||
      arm->command.plan_type == EC_PLAN_CIRCULAR) {
    return arm->planner_segment_index >= 1u;
  }
  return true;
}
