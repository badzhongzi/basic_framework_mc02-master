#include "ins_planner.h"

#include <math.h>
#include <string.h>

bool ins_planner_make_joint_command(
    EC_ArmFsmCmd *command, const float target[INS_ARM_6DOF_COUNT],
    float duration) {
  uint8_t i;
  if (!command || !target || duration < 0.0f) return false;

  memset(command, 0, sizeof(*command));
  command->plan_type = EC_PLAN_JOINT;
  command->node_count = 1;
  command->nodes[0].workspace = EC_WORKSPACE_JOINT;
  command->nodes[0].joint_count = INS_ARM_6DOF_COUNT;
  for (i = 0; i < INS_ARM_6DOF_COUNT; ++i) {
    command->nodes[0].joint[i] = target[i];
  }
  command->traj_time[0] = duration;
  return true;
}

bool ins_planner_make_multinode_joint_command(
    EC_ArmFsmCmd *command,
    const float nodes[][INS_ARM_6DOF_COUNT], const float traj_time[],
    uint8_t node_count) {
  uint8_t i;
  uint8_t j;

  if (!command || !nodes || !traj_time || node_count == 0 ||
      node_count > INS_PLANNER_MAX_NODES) {
    return false;
  }
  for (i = 0; i < node_count; ++i) {
    if (!isfinite(traj_time[i]) || traj_time[i] < 0.0f) return false;
    for (j = 0; j < INS_ARM_6DOF_COUNT; ++j) {
      if (!isfinite(nodes[i][j])) return false;
    }
  }

  memset(command, 0, sizeof(*command));
  command->plan_type = EC_PLAN_JOINT;
  command->node_count = node_count;
  for (i = 0; i < node_count; ++i) {
    command->nodes[i].workspace = EC_WORKSPACE_JOINT;
    command->nodes[i].joint_count = INS_ARM_6DOF_COUNT;
    command->traj_time[i] = traj_time[i];
    for (j = 0; j < INS_ARM_6DOF_COUNT; ++j) {
      command->nodes[i].joint[j] = nodes[i][j];
    }
  }
  return true;
}

bool ins_planner_make_none_command(EC_ArmFsmCmd *command,
                                   const EC_ArmCmd *node) {
  if (!command || !node) return false;

  memset(command, 0, sizeof(*command));
  command->plan_type = EC_PLAN_NONE;
  command->node_count = 1;
  command->nodes[0] = *node;
  return true;
}

bool ins_planner_make_none_joint_command(
    EC_ArmFsmCmd *command,
    const float target[INS_ARM_6DOF_COUNT]) {
  EC_ArmCmd node;
  uint8_t i;

  if (!command || !target) return false;
  memset(&node, 0, sizeof(node));
  node.workspace = EC_WORKSPACE_JOINT;
  node.joint_count = INS_ARM_6DOF_COUNT;
  for (i = 0; i < INS_ARM_6DOF_COUNT; ++i) {
    if (!isfinite(target[i])) return false;
    node.joint[i] = target[i];
  }
  return ins_planner_make_none_command(command, &node);
}

bool ins_planner_make_none_cartesian_command(EC_ArmFsmCmd *command,
                                             EC_CartesianCmd target) {
  EC_ArmCmd node;
  uint8_t i;
  float norm_squared = 0.0f;

  if (!command) return false;
  for (i = 0; i < 3; ++i) {
    if (!isfinite(target.pos.v[i])) return false;
  }
  for (i = 0; i < 4; ++i) {
    if (!isfinite(target.quat.v[i])) return false;
    norm_squared += target.quat.v[i] * target.quat.v[i];
  }
  if (!isfinite(norm_squared) || norm_squared <= 1.0e-12f) return false;

  memset(&node, 0, sizeof(node));
  node.workspace = EC_WORKSPACE_CARTESIAN;
  node.cartesian = target;
  return ins_planner_make_none_command(command, &node);
}

bool ins_planner_make_linear_command(EC_ArmFsmCmd *command,
                                     EC_CartesianCmd start,
                                     EC_CartesianCmd end, float duration) {
  if (!command || duration < 0.0f) return false;

  memset(command, 0, sizeof(*command));
  command->plan_type = EC_PLAN_LINEAR;
  command->node_count = 2;
  command->nodes[0].workspace = EC_WORKSPACE_CARTESIAN;
  command->nodes[0].cartesian = start;
  command->nodes[1].workspace = EC_WORKSPACE_CARTESIAN;
  command->nodes[1].cartesian = end;
  /* 笛卡尔直线模式只使用 traj_time[0]。 */
  command->traj_time[0] = duration;
  return true;
}
