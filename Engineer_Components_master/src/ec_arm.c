#include "ec_arm.h"

#include <math.h>
#include <string.h>

#define EC_ARM_MAX_NODES 8u

static void arm_pose_update(EC_Arm *arm) {
  uint8_t i;
  float joint[EC_MAX_JOINTS];

  if (!arm || !arm->serial) return;
  for (i = 0; i < arm->joint_count; ++i) joint[i] = arm->joint_feedback[i];
  if (!ec_serial_link_fkine(arm->serial, joint)) return;

  arm->pose.joint_count = arm->joint_count;
  for (i = 0; i < arm->joint_count; ++i) arm->pose.joint[i] = joint[i];
  arm->pose.matrix = ec_serial_link_get_t(arm->serial);
  arm->pose.cartesian.pos = ec_serial_link_get_p(arm->serial);
  arm->pose.cartesian.quat = ec_serial_link_get_quat(arm->serial);
}

static uint8_t choose_solution(
    const EC_Arm *arm, uint8_t count,
    float solution[EC_MAX_IK_SOLUTIONS][EC_MAX_JOINTS],
    const float *reference, float *out) {
  uint8_t i;
  uint8_t j;
  uint8_t best = 0;
  float best_dist = 1.0e30f;

  if (!arm || !reference || !out) return 0;
  if (count > EC_MAX_IK_SOLUTIONS) count = EC_MAX_IK_SOLUTIONS;

  for (i = 0; i < count; ++i) {
    float distance = 0.0f;
    bool valid = true;

    for (j = 0; j < arm->joint_count; ++j) {
      float q = ec_norm_angle(solution[i][j]);
      float delta = q - reference[j];
      if (!isfinite(q) || q < arm->joint_min[j] || q > arm->joint_max[j]) {
        valid = false;
        break;
      }
      distance += delta * delta;
    }
    if (valid && distance < best_dist) {
      best_dist = distance;
      best = i;
    }
  }

  if (best_dist >= 1.0e29f) return 0;
  for (j = 0; j < arm->joint_count; ++j) {
    out[j] = ec_norm_angle(solution[best][j]);
  }
  return 1;
}

static bool solve_cartesian_target(const EC_Arm *arm,
                                   EC_CartesianCmd cartesian,
                                   const float *free_joints,
                                   const float *reference, float *out) {
  uint8_t count;
  float solution[EC_MAX_IK_SOLUTIONS][EC_MAX_JOINTS];

  if (!arm || !arm->solve_ikine) return false;
  count = arm->solve_ikine(cartesian, free_joints, arm->joint_count,
                           solution, arm->user);
  return choose_solution(arm, count, solution, reference, out) != 0;
}

static void clamp_reference(EC_Arm *arm) {
  uint8_t i;
  for (i = 0; i < arm->joint_count; ++i) {
    float before = arm->joint_reference[i];
    arm->joint_reference[i] =
        ec_limit(before, arm->joint_min[i], arm->joint_max[i]);
    if (arm->joint_reference[i] != before) {
      arm->joint_velocity_reference[i] = 0.0f;
      arm->joint_acceleration_reference[i] = 0.0f;
      arm->previous_reference[i] = arm->joint_reference[i];
      arm->previous_velocity_reference[i] = 0.0f;
    }
  }
}

static void update_state_reference(EC_Arm *arm) {
  uint8_t i;
  if (!arm) return;
  arm->state_reference.joint_count = arm->joint_count;
  for (i = 0; i < arm->joint_count; ++i) {
    arm->state_reference.q[i] = arm->joint_reference[i];
    arm->state_reference.q_dot[i] = arm->joint_velocity_reference[i];
    arm->state_reference.q_ddot[i] =
        arm->joint_acceleration_reference[i];
  }
}

static float shortest_angle_delta(float current, float previous) {
  float delta = current - previous;
  while (delta > EC_PI) delta -= 2.0f * EC_PI;
  while (delta < -EC_PI) delta += 2.0f * EC_PI;
  return delta;
}

static void update_cartesian_state(EC_Arm *arm) {
  uint8_t i;
  float period;
  if (!arm) return;
  period = arm->cartesian_planner.ctrl_period;
  if (!arm->state_history_valid || period <= 0.0f) {
    for (i = 0; i < arm->joint_count; ++i) {
      arm->joint_velocity_reference[i] = 0.0f;
      arm->joint_acceleration_reference[i] = 0.0f;
      arm->previous_reference[i] = arm->joint_reference[i];
      arm->previous_velocity_reference[i] = 0.0f;
    }
    arm->state_history_valid = true;
    return;
  }
  for (i = 0; i < arm->joint_count; ++i) {
    float velocity = shortest_angle_delta(arm->joint_reference[i],
                                          arm->previous_reference[i]) / period;
    arm->joint_acceleration_reference[i] =
        (velocity - arm->previous_velocity_reference[i]) / period;
    arm->joint_velocity_reference[i] = velocity;
    arm->previous_reference[i] = arm->joint_reference[i];
    arm->previous_velocity_reference[i] = velocity;
  }
}

static bool valid_workspace(EC_Workspace workspace) {
  return workspace == EC_WORKSPACE_JOINT ||
         workspace == EC_WORKSPACE_CARTESIAN;
}

static bool valid_joint_node(const EC_Arm *arm, const EC_ArmCmd *node) {
  uint8_t i;
  if (node->workspace != EC_WORKSPACE_JOINT) return true;
  if (node->joint_count != arm->joint_count) return false;
  for (i = 0; i < arm->joint_count; ++i) {
    if (!isfinite(node->joint[i])) return false;
  }
  return true;
}

static bool valid_cartesian_pose(EC_CartesianCmd pose) {
  uint8_t i;
  float norm_squared = 0.0f;
  for (i = 0; i < 3; ++i) {
    if (!isfinite(pose.pos.v[i])) return false;
  }
  for (i = 0; i < 4; ++i) {
    if (!isfinite(pose.quat.v[i])) return false;
    norm_squared += pose.quat.v[i] * pose.quat.v[i];
  }
  return isfinite(norm_squared) && norm_squared > 1.0e-12f;
}

static bool valid_circle_geometry(EC_CartesianCmd start,
                                  EC_CartesianCmd end, EC_Vec3 center) {
  EC_Vec3 start_radius = ec_vec3_sub(start.pos, center);
  EC_Vec3 end_radius = ec_vec3_sub(end.pos, center);
  float start_norm = ec_vec3_norm(start_radius);
  float end_norm = ec_vec3_norm(end_radius);
  float scale = fmaxf(start_norm, end_norm);
  uint8_t i;

  for (i = 0; i < 3; ++i) {
    if (!isfinite(center.v[i])) return false;
  }
  if (start_norm <= 1.0e-6f || end_norm <= 1.0e-6f ||
      fabsf(start_norm - end_norm) > 1.0e-4f * (1.0f + scale)) {
    return false;
  }
  return ec_vec3_norm(ec_vec3_cross(start_radius, end_radius)) >
         1.0e-6f * start_norm * end_norm;
}

static bool valid_traj_time(float time) {
  return isfinite(time) && time >= 0.0f;
}

static bool validate_command(const EC_Arm *arm,
                             const EC_ArmFsmCmd *command) {
  uint8_t i;

  if (!arm || !command || !arm->initialized || command->node_count == 0 ||
      command->node_count > EC_ARM_MAX_NODES ||
      command->plan_type < EC_PLAN_JOINT ||
      command->plan_type > EC_PLAN_NONE) {
    return false;
  }

  for (i = 0; i < command->node_count; ++i) {
    if (!valid_workspace(command->nodes[i].workspace) ||
        !valid_joint_node(arm, &command->nodes[i])) {
      return false;
    }
  }

  if (command->plan_type == EC_PLAN_JOINT) {
    for (i = 0; i < command->node_count; ++i) {
      if (!valid_traj_time(command->traj_time[i])) return false;
      if (command->nodes[i].workspace == EC_WORKSPACE_CARTESIAN &&
          (!arm->solve_ikine ||
           !valid_cartesian_pose(command->nodes[i].cartesian))) {
        return false;
      }
    }
    return true;
  }

  if (command->plan_type == EC_PLAN_LINEAR) {
    return command->node_count >= 2 && arm->solve_ikine &&
           command->nodes[0].workspace == EC_WORKSPACE_CARTESIAN &&
           command->nodes[1].workspace == EC_WORKSPACE_CARTESIAN &&
           valid_cartesian_pose(command->nodes[0].cartesian) &&
           valid_cartesian_pose(command->nodes[1].cartesian) &&
           valid_traj_time(command->traj_time[0]);
  }

  if (command->plan_type == EC_PLAN_CIRCULAR) {
    return command->node_count >= 3 && arm->solve_ikine &&
           command->nodes[0].workspace == EC_WORKSPACE_CARTESIAN &&
           command->nodes[1].workspace == EC_WORKSPACE_CARTESIAN &&
           command->nodes[2].workspace == EC_WORKSPACE_CARTESIAN &&
           valid_cartesian_pose(command->nodes[0].cartesian) &&
           valid_cartesian_pose(command->nodes[1].cartesian) &&
           valid_circle_geometry(command->nodes[0].cartesian,
                                 command->nodes[1].cartesian,
                                 command->nodes[2].cartesian.pos) &&
           valid_traj_time(command->traj_time[0]);
  }

  return command->nodes[0].workspace == EC_WORKSPACE_JOINT ||
         (arm->solve_ikine != NULL &&
          valid_cartesian_pose(command->nodes[0].cartesian));
}

bool ec_arm_init(EC_Arm *arm, uint8_t joint_count, EC_SerialLink *serial,
                 ec_solve_ikine_fn solve_ikine, void *user) {
  uint8_t i;

  if (!arm || !serial || joint_count == 0 || joint_count > EC_MAX_JOINTS ||
      serial->count < joint_count) {
    return false;
  }

  memset(arm, 0, sizeof(*arm));
  arm->joint_count = joint_count;
  arm->state_reference.joint_count = joint_count;
  arm->serial = serial;
  arm->solve_ikine = solve_ikine;
  arm->user = user;
  for (i = 0; i < joint_count; ++i) {
    arm->joint_min[i] = serial->links[i].mdh.qmin;
    arm->joint_max[i] = serial->links[i].mdh.qmax;
  }
  ec_joint_planner_init(&arm->joint_planner, 0.001f, joint_count);
  ec_cartesian_planner_init(&arm->cartesian_planner, 0.001f);
  arm->initialized = true;
  return true;
}

bool ec_arm_set_control_period(EC_Arm *arm, float control_period) {
  if (!arm || !arm->initialized || !isfinite(control_period) ||
      control_period <= 0.0f) {
    return false;
  }
  arm->joint_planner.ctrl_period = control_period;
  arm->cartesian_planner.ctrl_period = control_period;
  return true;
}

void ec_arm_set_feedback(EC_Arm *arm, const float *joint) {
  uint8_t i;
  if (!arm || !joint) return;
  for (i = 0; i < arm->joint_count; ++i) arm->joint_feedback[i] = joint[i];
}

void ec_arm_set_velocity_feedback(EC_Arm *arm, const float *joint_velocity) {
  uint8_t i;
  if (!arm || !joint_velocity) return;
  for (i = 0; i < arm->joint_count; ++i) {
    arm->joint_velocity_feedback[i] = joint_velocity[i];
  }
}

bool ec_arm_set_state_reference(EC_Arm *arm,
                               const EC_JointStateReference *state) {
  uint8_t i;

  if (!arm || !arm->initialized || !state) return false;
  if (state->joint_count != arm->joint_count) return false;

  for (i = 0; i < arm->joint_count; ++i) {
    arm->joint_reference[i] = state->q[i];
    arm->joint_velocity_reference[i] = state->q_dot[i];
    arm->joint_acceleration_reference[i] = state->q_ddot[i];
  }
  update_state_reference(arm);
  return true;
}

void ec_arm_update(EC_Arm *arm) {
  if (arm && arm->initialized) arm_pose_update(arm);
}

bool ec_arm_set_command(EC_Arm *arm, const EC_ArmFsmCmd *command) {
  if (!validate_command(arm, command)) return false;
  arm->command = *command;
  arm->planner_active = false;
  arm->planner_segment_index = 0;
  arm->state_history_valid = false;
  memset(arm->joint_velocity_reference, 0,
         sizeof(arm->joint_velocity_reference));
  memset(arm->joint_acceleration_reference, 0,
         sizeof(arm->joint_acceleration_reference));
  update_state_reference(arm);
  return true;
}

static bool start_joint_segment(EC_Arm *arm) {
  const EC_ArmCmd *node;
  const float *start;
  float target[EC_MAX_JOINTS];
  uint8_t i;

  if (arm->planner_segment_index >= arm->command.node_count) return true;

  /* Match C++ MultinodeJointPlanner: feedback is the implicit path start;
   * each command node is the end of one segment. */
  node = &arm->command.nodes[arm->planner_segment_index];
  start = arm->planner_segment_index == 0 ? arm->joint_feedback
                                          : arm->joint_planner.end;
  if (node->workspace == EC_WORKSPACE_JOINT) {
    for (i = 0; i < arm->joint_count; ++i) target[i] = node->joint[i];
  } else if (!solve_cartesian_target(arm, node->cartesian, node->joint,
                                     start, target)) {
    return false;
  }

  if (!ec_joint_planner_set(
          &arm->joint_planner, start, target,
          arm->command.traj_time[arm->planner_segment_index])) {
    return false;
  }
  arm->planner_active = true;
  return true;
}

static bool generate_joint_reference(EC_Arm *arm) {
  if (arm->planner_segment_index >= arm->command.node_count) return true;
  if (!arm->planner_active && !start_joint_segment(arm)) return false;

  ec_joint_planner_plan_state(
      &arm->joint_planner, arm->joint_reference,
      arm->joint_velocity_reference, arm->joint_acceleration_reference,
      EC_MAX_JOINTS);
  clamp_reference(arm);
  update_state_reference(arm);
  if (ec_joint_planner_finished(&arm->joint_planner)) {
    arm->planner_active = false;
    ++arm->planner_segment_index;
  }
  return true;
}

static void start_cartesian_segment(EC_Arm *arm) {
  /* Match ModuleArm::cartesianPlan: one fixed Cartesian segment is built from
   * nodes[0..1] (and nodes[2] as circle center); later nodes are ignored. */
  if (arm->command.plan_type == EC_PLAN_LINEAR) {
    ec_cartesian_planner_set_linear(
        &arm->cartesian_planner, arm->command.nodes[0].cartesian,
        arm->command.nodes[1].cartesian, arm->command.traj_time[0]);
  } else {
    ec_cartesian_planner_set_circle(
        &arm->cartesian_planner, arm->command.nodes[0].cartesian,
        arm->command.nodes[1].cartesian,
        arm->command.nodes[2].cartesian.pos, arm->command.traj_time[0], false);
  }
  arm->planner_active = true;
}

static bool generate_cartesian_reference(EC_Arm *arm) {
  EC_CartesianCmd cartesian;
  EC_CartesianPlanner planner_before_plan;

  if (arm->planner_segment_index >= 1) {
    memset(arm->joint_velocity_reference, 0,
           sizeof(arm->joint_velocity_reference));
    memset(arm->joint_acceleration_reference, 0,
           sizeof(arm->joint_acceleration_reference));
    update_state_reference(arm);
    return true;
  }
  if (!arm->planner_active) start_cartesian_segment(arm);

  planner_before_plan = arm->cartesian_planner;
  cartesian = ec_cartesian_planner_plan(&arm->cartesian_planner);
  if (!solve_cartesian_target(
          arm, cartesian, NULL,
          arm->state_history_valid ? arm->previous_reference
                                   : arm->joint_feedback,
                              arm->joint_reference)) {
    arm->cartesian_planner = planner_before_plan;
    return false;
  }
  update_cartesian_state(arm);
  clamp_reference(arm);
  update_state_reference(arm);
  if (ec_cartesian_planner_finished(&arm->cartesian_planner)) {
    memset(arm->joint_velocity_reference, 0,
           sizeof(arm->joint_velocity_reference));
    memset(arm->joint_acceleration_reference, 0,
           sizeof(arm->joint_acceleration_reference));
    memset(arm->previous_velocity_reference, 0,
           sizeof(arm->previous_velocity_reference));
    update_state_reference(arm);
    arm->planner_active = false;
    arm->planner_segment_index = 1;
  }
  return true;
}

static bool generate_unplanned_reference(EC_Arm *arm) {
  const EC_ArmCmd *node = &arm->command.nodes[0];
  uint8_t i;

  if (node->workspace == EC_WORKSPACE_CARTESIAN) {
    if (!solve_cartesian_target(arm, node->cartesian, NULL,
                                arm->joint_feedback,
                                arm->joint_reference)) {
      return false;
    }
  } else {
    for (i = 0; i < arm->joint_count; ++i) {
      arm->joint_reference[i] = node->joint[i];
    }
  }
  memset(arm->joint_velocity_reference, 0,
         sizeof(arm->joint_velocity_reference));
  memset(arm->joint_acceleration_reference, 0,
         sizeof(arm->joint_acceleration_reference));
  clamp_reference(arm);
  update_state_reference(arm);
  return true;
}

bool ec_arm_generate_reference(EC_Arm *arm) {
  if (!arm || !arm->initialized || arm->command.node_count == 0) return false;

  if (arm->command.plan_type == EC_PLAN_JOINT) {
    return generate_joint_reference(arm);
  }
  if (arm->command.plan_type == EC_PLAN_LINEAR ||
      arm->command.plan_type == EC_PLAN_CIRCULAR) {
    return generate_cartesian_reference(arm);
  }
  if (arm->command.plan_type == EC_PLAN_NONE) {
    return generate_unplanned_reference(arm);
  }
  return false;
}

const EC_ArmPose *ec_arm_pose(const EC_Arm *arm) {
  return arm ? &arm->pose : NULL;
}

const float *ec_arm_reference(const EC_Arm *arm) {
  return arm ? arm->joint_reference : NULL;
}

const EC_JointStateReference *ec_arm_state_reference(const EC_Arm *arm) {
  return arm ? &arm->state_reference : NULL;
}
