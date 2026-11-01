#include "ins_arm.h"

#include <string.h>

/*
 * ikine5 使用的几何参数，单位 m。
 *
 * 注意: 这三个长度是旧机械臂的尺寸, 和下面按新 URDF 推出来的 MDH 模型
 * 不一致. 逆解当前解不出来、笛卡尔规划也未启用, 所以先不动; 等逆解
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
   * 参数来自真实机械臂的 URDF (D:/rm/urdf_urdf/urdf.urdf), 和 MPC 生成代码
   * (Generated/rnea_6axis.c 等) 用的是同一个模型. 换 URDF 时两边一起换.
   *
   * 约定: 这里的关节角 q 就是 URDF 的关节角 (同一个零点、同一个正方向).
   * MDH 的 z_i 沿 URDF 的 axis 方向 (J0、J1、J2 的 axis 是 -z / -y, 所以有
   * alpha = pi 和 offset), q = 0 时 MDH 正解和 URDF 零位姿态一致, 零位差全部
   * 放在 offset 里. 前馈用的 rnea() 吃的就是这个 q, 这条约定不能破.
   *
   * 由 URDF 推 Craig (modified DH) 的过程和校验: 2000 组随机构型下每个关节的
   * 原点、转轴和 URDF 吻合 (偏差 1e-15), MDH 第 6 系和 URDF link6 系完全重合
   * (位置和姿态都一样, 没有工具系旋转). base 系就是 MDH 第 0 系, 重力沿 -z0.
   *
   * 参数顺序: offset(rad), d(m), a(m), alpha(rad); 其中 d 为 d_i,
   * offset 为 theta_i 偏置, a/alpha 实为 a_{i-1}/alpha_{i-1}.
   */
  static const float mdh[INS_ARM_6DOF_COUNT][4] = {
      {3.141592654f, -0.130500000f, 0.000000000f, 3.141592654f},
      {-1.530474588f, 0.007445710f, 0.000000000f, 1.570796327f},
      {1.530474588f, 0.020635020f, 0.405000009f, 0.000000000f},
      {3.141592654f, 0.434003090f, 0.071585030f, 1.570796327f},
      {0.000000000f, 0.000000000f, 0.002081680f, -1.570796327f},
      {0.000000000f, -0.014599270f, 0.000145880f, 1.570796327f},
  };
  /* URDF 的限位, 六个关节都是 +-3.14 rad (负责人要求先写死). */
  static const float qmin[INS_ARM_6DOF_COUNT] = {
      -3.14f, -3.14f, -3.14f, -3.14f, -3.14f, -3.14f,
  };
  static const float qmax[INS_ARM_6DOF_COUNT] = {
      3.14f, 3.14f, 3.14f, 3.14f, 3.14f, 3.14f,
  };
  /* URDF link1..link6 的质量, kg. 不含负载和转子惯量 (armature 只在生成的 RNEA 里). */
  static const float mass[INS_ARM_6DOF_COUNT] = {
      1.16589812f, 1.45300263f, 0.78687754f, 0.56631370f, 0.48278546f, 0.57709208f,
  };
  /*
   * 质心表达在 MDH 连杆系 (ec_serial_link_inverse_dynamics 中按
   * body_origin[i] + R_i * centroid 使用), 由 URDF 的 inertial origin 换算过来.
   */
  static const EC_Vec3 centroid[INS_ARM_6DOF_COUNT] = {
      {{0.000007960f, -0.019564960f, 0.011285200f}},
      {{0.328235519f, 0.000010047f, 0.018181400f}},
      {{0.056044320f, -0.175341240f, -0.001150370f}},
      {{0.001658640f, -0.069821840f, -0.013858970f}},
      {{0.000045240f, -0.000194960f, -0.003078850f}},
      {{-0.001018940f, 0.000036560f, 0.100232280f}},
  };
  /* 绕质心的转动惯量, 表达在 MDH 连杆系 (行主序), 由 URDF 的 inertia 旋转过来. */
  static const EC_Mat3 inertia[INS_ARM_6DOF_COUNT] = {
      {{1.505810000e-03f, -2.200000000e-07f, 6.000000000e-08f,
        -2.200000000e-07f, 1.332640000e-03f, -2.668500000e-04f,
        6.000000000e-08f, -2.668500000e-04f, 9.885600000e-04f}},
      {{1.237551021e-03f, -1.021730502e-06f, -1.838273990e-03f,
        -1.021730502e-06f, 2.923000898e-02f, 2.525260890e-07f,
        -1.838273990e-03f, 2.525260890e-07f, 2.886547000e-02f}},
      {{1.981079000e-02f, 2.136100000e-03f, -2.623000000e-05f,
        2.136100000e-03f, 1.403110000e-03f, 1.677300000e-04f,
        -2.623000000e-05f, 1.677300000e-04f, 1.987640000e-02f}},
      {{9.877200000e-04f, 9.820000000e-06f, -1.269000000e-05f,
        9.820000000e-06f, 6.140500000e-04f, 2.448300000e-04f,
        -1.269000000e-05f, 2.448300000e-04f, 6.566600000e-04f}},
      {{3.943800000e-04f, -7.900000000e-07f, 0.000000000e+00f,
        -7.900000000e-07f, 3.583100000e-04f, 7.900000000e-07f,
        0.000000000e+00f, 7.900000000e-07f, 2.861500000e-04f}},
      {{1.047530000e-03f, -4.610000000e-05f, 5.470000000e-06f,
        -4.610000000e-05f, 9.958500000e-04f, -2.100000000e-06f,
        5.470000000e-06f, -2.100000000e-06f, 3.410200000e-04f}},
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
