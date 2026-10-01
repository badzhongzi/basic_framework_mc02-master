#include "ec_link.h"

#include <math.h>
#include <string.h>

void ec_mdh_init(EC_MDH *mdh, float offset, float d, float a, float alpha,
                float qmin, float qmax) {
  if (!mdh) return;
  mdh->offset=offset; mdh->d=d; mdh->a=a; mdh->alpha=alpha;
  mdh->qmin=qmin; mdh->qmax=qmax;
}

EC_Mat4 ec_mdh_forward(const EC_MDH *mdh, float q) {
  return ec_mdh_to_transform(q + (mdh ? mdh->offset : 0.0f), mdh ? mdh->d : 0,
                              mdh ? mdh->a : 0, mdh ? mdh->alpha : 0);
}
EC_Mat4 ec_mdh_backward(const EC_MDH *mdh, float q) {
  return ec_mat4_inverse(ec_mdh_forward(mdh, q));
}

void ec_link_init(EC_Link *link, const EC_MDH *mdh) {
  if (!link) return;
  memset(link,0,sizeof(*link));
  if (mdh) link->mdh=*mdh;
  else ec_mdh_init(&link->mdh,0,0,0,0,-EC_PI,EC_PI);
  ec_mat3_zero(&link->dynamic.inertia);
}
void ec_link_set_dynamic(EC_Link *link, float mass, EC_Vec3 centroid, EC_Mat3 inertia) {
  if (!link) return;
  link->dynamic.mass=mass; link->dynamic.centroid=centroid; link->dynamic.inertia=inertia;
}

void ec_serial_link_init(EC_SerialLink *serial) {
  if (!serial) return;
  memset(serial,0,sizeof(*serial)); ec_mat4_identity(&serial->fkine);
}
bool ec_serial_link_append(EC_SerialLink *serial, const EC_Link *link) {
  if (!serial || !link || serial->count >= EC_MAX_JOINTS) return false;
  serial->links[serial->count++]=*link; return true;
}
bool ec_serial_link_fkine(EC_SerialLink *serial, const float *theta) {
  uint16_t i; EC_Mat4 t;
  if (!serial || !theta) return false;
  ec_mat4_identity(&t);
  for (i=0;i<serial->count;++i) { serial->transforms[i]=ec_mdh_forward(&serial->links[i].mdh,theta[i]); t=ec_mat4_mul(t,serial->transforms[i]); }
  serial->fkine=t; return true;
}
EC_Mat4 ec_serial_link_get_t(const EC_SerialLink *serial) { EC_Mat4 t; ec_mat4_identity(&t); if(serial) t=serial->fkine; return t; }
EC_Mat3 ec_serial_link_get_r(const EC_SerialLink *serial) { return ec_t_to_r(ec_serial_link_get_t(serial)); }
EC_Vec3 ec_serial_link_get_p(const EC_SerialLink *serial) { return ec_t_to_p(ec_serial_link_get_t(serial)); }
EC_Quat ec_serial_link_get_quat(const EC_SerialLink *serial) { return ec_t_to_quat(ec_serial_link_get_t(serial)); }
bool ec_serial_link_set_mdh(EC_SerialLink *serial, uint8_t index,float offset,float d,float a,float alpha) {
  if (!serial || index>=serial->count) return false;
  { float qmin=serial->links[index].mdh.qmin,qmax=serial->links[index].mdh.qmax; ec_mdh_init(&serial->links[index].mdh,offset,d,a,alpha,qmin,qmax); }
  return true;
}
bool ec_serial_link_get_mdh(const EC_SerialLink *serial,uint8_t index,EC_MDH *mdh) { if(!serial||!mdh||index>=serial->count)return false;*mdh=serial->links[index].mdh;return true; }
bool ec_serial_link_set_limit(EC_SerialLink *serial,uint8_t index,float qmin,float qmax) { if(!serial||index>=serial->count||qmin>qmax)return false;serial->links[index].mdh.qmin=qmin;serial->links[index].mdh.qmax=qmax;return true; }
bool ec_serial_link_get_limit(const EC_SerialLink *serial,uint8_t index,float *qmin,float *qmax) { if(!serial||index>=serial->count||!qmin||!qmax)return false;*qmin=serial->links[index].mdh.qmin;*qmax=serial->links[index].mdh.qmax;return true; }
bool ec_serial_link_set_dynamic(EC_SerialLink *serial,uint8_t index,float mass,EC_Vec3 centroid,EC_Mat3 inertia) { if(!serial||index>=serial->count)return false;ec_link_set_dynamic(&serial->links[index],mass,centroid,inertia);return true; }
bool ec_serial_link_get_dynamic(const EC_SerialLink *serial,uint8_t index,EC_DynamicParam *dynamic) { if(!serial||index>=serial->count||!dynamic)return false;*dynamic=serial->links[index].dynamic;return true; }

bool ec_serial_link_jacobian(const EC_SerialLink *serial, float *out, size_t out_count) {
  EC_Mat4 end[EC_MAX_JOINTS]; EC_Vec3 p_end; uint16_t i; size_t required;
  if(!serial||!out||serial->count==0)return false;
  required=(size_t)serial->count*6;
  if(out_count<required)return false;
  for(i=0;i<serial->count;++i) end[i]=(i==0)?serial->transforms[i]:ec_mat4_mul(end[i-1],serial->transforms[i]);
  p_end=ec_t_to_p(end[serial->count-1]);
  for(i=0;i<serial->count;++i) { EC_Vec3 z=ec_vec3(end[i].v[2],end[i].v[6],end[i].v[10]); EC_Vec3 p=ec_t_to_p(end[i]); EC_Vec3 jv=ec_vec3_cross(z,ec_vec3_sub(p_end,p)); out[i]=jv.v[0];out[serial->count+i]=jv.v[1];out[2*serial->count+i]=jv.v[2];out[3*serial->count+i]=z.v[0];out[4*serial->count+i]=z.v[1];out[5*serial->count+i]=z.v[2]; }
  return true;
}

static void gravity_forward_pass(const EC_SerialLink *serial,
                                 const float *theta,
                                 EC_Mat4 *transform,
                                 EC_Vec3 *joint_axis,
                                 EC_Vec3 *joint_origin,
                                 EC_Vec3 *centroid_world) {
  uint16_t i;
  for (i = 0; i < serial->count; ++i) {
    EC_Mat4 local = ec_mdh_forward(&serial->links[i].mdh, theta[i]);
    transform[i] = i == 0 ? local : ec_mat4_mul(transform[i - 1], local);
    joint_axis[i] = ec_vec3(transform[i].v[2], transform[i].v[6],
                            transform[i].v[10]);
    joint_origin[i] = ec_t_to_p(transform[i]);
    centroid_world[i] = ec_vec3_add(
        joint_origin[i],
        ec_mat3_vec(ec_t_to_r(transform[i]),
                    serial->links[i].dynamic.centroid));
  }
}

#if EC_GRAVITY_METHOD != EC_GRAVITY_NEWTON_EULER
static void gravity_jacobian_transpose(const EC_SerialLink *serial,
                                       float *torque,
                                       const EC_Vec3 *joint_axis,
                                       const EC_Vec3 *joint_origin,
                                       const EC_Vec3 *centroid_world) {
  uint16_t i, link_index;
  for (i = 0; i < serial->count; ++i) {
    for (link_index = i; link_index < serial->count; ++link_index) {
      EC_Vec3 upward_force =
          ec_vec3(0.0f, 0.0f,
                  9.81f * serial->links[link_index].dynamic.mass);
      EC_Vec3 lever = ec_vec3_sub(centroid_world[link_index], joint_origin[i]);
      torque[i] += ec_vec3_dot(
          joint_axis[i], ec_vec3_cross(lever, upward_force));
    }
  }
}
#endif

#if EC_GRAVITY_METHOD == EC_GRAVITY_NEWTON_EULER
static void gravity_newton_euler(const EC_SerialLink *serial,
                                 float *torque,
                                 const EC_Vec3 *joint_axis,
                                 const EC_Vec3 *joint_origin,
                                 const EC_Vec3 *centroid_world) {
  EC_Vec3 force_next = ec_vec3(0.0f, 0.0f, 0.0f);
  EC_Vec3 moment_next = ec_vec3(0.0f, 0.0f, 0.0f);
  uint16_t i;

  /*
   * 这里的 force_next 和 moment_next 分别表示所有已处理连杆的合力，
   * 以及该合力/力矩关于下一个关节原点的合力矩。反向移动到当前关节
   * 原点时，用 r × F 平移力矩；加入当前连杆质心的重力后即可投影到
   * 当前关节轴。所有量都在世界坐标系中，因此不需要额外的坐标变换。
   * 该递推每个连杆只访问一次，时间复杂度为 O(N)。
   *
   * 这里使用 +Z 方向的等效支撑力 9.81 m/s²；它对应重力沿 -Z 时的
   * 补偿力矩，和 gravity_jacobian_transpose() 的输出符号保持一致。
   */
  for (i = serial->count; i-- > 0;) {
    const float mass = serial->links[i].dynamic.mass;
    const EC_Vec3 force_own = ec_vec3(0.0f, 0.0f, 9.81f * mass);
    const EC_Vec3 lever_own =
        ec_vec3_sub(centroid_world[i], joint_origin[i]);
    EC_Vec3 moment = ec_vec3_cross(lever_own, force_own);

    if (i + 1u < serial->count) {
      const EC_Vec3 lever_next =
          ec_vec3_sub(joint_origin[i + 1u], joint_origin[i]);
      moment = ec_vec3_add(moment,
                           ec_vec3_add(moment_next,
                                       ec_vec3_cross(lever_next, force_next)));
    } else {
      moment = ec_vec3_add(moment, moment_next);
    }

    force_next = ec_vec3_add(force_next, force_own);
    moment_next = moment;
    torque[i] = ec_vec3_dot(joint_axis[i], moment);
  }
}
#endif

void ec_serial_link_gravity(const EC_SerialLink *serial,
                            const float *theta, float *torque) {
  EC_Mat4 transform[EC_MAX_JOINTS];
  EC_Vec3 joint_axis[EC_MAX_JOINTS];
  EC_Vec3 joint_origin[EC_MAX_JOINTS];
  EC_Vec3 centroid_world[EC_MAX_JOINTS];

  if (!serial || !theta || !torque) return;
  if (serial->count == 0 || serial->count > EC_MAX_JOINTS) return;
  memset(torque, 0, sizeof(float) * serial->count);

  gravity_forward_pass(serial, theta, transform,
                        joint_axis, joint_origin, centroid_world);

#if EC_GRAVITY_METHOD == EC_GRAVITY_NEWTON_EULER
  gravity_newton_euler(serial, torque, joint_axis, joint_origin,
                       centroid_world);
#else
  gravity_jacobian_transpose(serial, torque, joint_axis, joint_origin,
                             centroid_world);
#endif
}

bool ec_serial_link_inverse_dynamics(const EC_SerialLink *serial,
                                     const float *q, const float *q_dot,
                                     const float *q_ddot, float *torque) {
  EC_Mat4 transform[EC_MAX_JOINTS];
  EC_Vec3 axis[EC_MAX_JOINTS];
  EC_Vec3 joint_origin[EC_MAX_JOINTS];
  EC_Vec3 body_origin[EC_MAX_JOINTS];
  EC_Vec3 com[EC_MAX_JOINTS];
  EC_Vec3 omega[EC_MAX_JOINTS];
  EC_Vec3 alpha[EC_MAX_JOINTS];
  EC_Vec3 origin_accel[EC_MAX_JOINTS];
  EC_Vec3 force[EC_MAX_JOINTS];
  EC_Vec3 moment[EC_MAX_JOINTS];
  EC_Mat3 rotation[EC_MAX_JOINTS];
  uint16_t i;
  const EC_Vec3 zero = ec_vec3(0.0f, 0.0f, 0.0f);
  const EC_Vec3 gravity_compensation_accel = ec_vec3(0.0f, 0.0f, 9.81f);

  if (!serial || !q || !q_dot || !q_ddot || !torque ||
      serial->count == 0 || serial->count > EC_MAX_JOINTS) {
    return false;
  }
  memset(torque, 0, sizeof(float) * serial->count);

  /* 前向递推均在基座坐标系中进行，避免在 MDH 局部坐标系之间混用向量。 */
  for (i = 0; i < serial->count; ++i) {
    EC_Mat4 parent_transform;
    EC_Mat4 joint_transform;
    EC_Vec3 parent_origin = i == 0 ? zero : body_origin[i - 1];
    EC_Vec3 parent_omega = i == 0 ? zero : omega[i - 1];
    EC_Vec3 parent_alpha = i == 0 ? zero : alpha[i - 1];
    EC_Vec3 r;
    EC_Vec3 joint_rate;
    EC_Vec3 joint_accel;

    parent_transform = i == 0 ? (EC_Mat4){{1.0f, 0.0f, 0.0f, 0.0f,
                                           0.0f, 1.0f, 0.0f, 0.0f,
                                           0.0f, 0.0f, 1.0f, 0.0f,
                                           0.0f, 0.0f, 0.0f, 1.0f}}
                              : transform[i - 1];
    joint_transform = ec_mat4_mul(
        parent_transform,
        ec_mdh_to_transform(0.0f, 0.0f, serial->links[i].mdh.a,
                            serial->links[i].mdh.alpha));
    transform[i] = ec_mat4_mul(
        parent_transform, ec_mdh_forward(&serial->links[i].mdh, q[i]));
    rotation[i] = ec_t_to_r(transform[i]);
    joint_origin[i] = ec_t_to_p(joint_transform);
    body_origin[i] = ec_t_to_p(transform[i]);
    r = ec_vec3_sub(joint_origin[i], parent_origin);
    axis[i] = ec_vec3(joint_transform.v[2], joint_transform.v[6],
                      joint_transform.v[10]);
    com[i] = ec_vec3_add(body_origin[i], ec_mat3_vec(rotation[i],
                                                serial->links[i].dynamic.centroid));

    joint_rate = ec_vec3_scale(axis[i], q_dot[i]);
    joint_accel = ec_vec3_scale(axis[i], q_ddot[i]);
    omega[i] = ec_vec3_add(parent_omega, joint_rate);
    alpha[i] = ec_vec3_add(
        ec_vec3_add(parent_alpha, joint_accel),
        ec_vec3_cross(parent_omega, joint_rate));
    origin_accel[i] = ec_vec3_add(
        ec_vec3_add(i == 0 ? gravity_compensation_accel
                            : origin_accel[i - 1],
                    ec_vec3_cross(parent_alpha, r)),
        ec_vec3_cross(parent_omega, ec_vec3_cross(parent_omega, r)));
    r = ec_vec3_sub(body_origin[i], joint_origin[i]);
    origin_accel[i] = ec_vec3_add(
        origin_accel[i],
        ec_vec3_add(ec_vec3_cross(alpha[i], r),
                    ec_vec3_cross(omega[i], ec_vec3_cross(omega[i], r))));
  }

  for (i = serial->count; i-- > 0;) {
    const EC_DynamicParam *dynamic = &serial->links[i].dynamic;
    EC_Vec3 r_com = ec_vec3_sub(com[i], body_origin[i]);
    EC_Vec3 com_accel = ec_vec3_add(
        ec_vec3_add(origin_accel[i], ec_vec3_cross(alpha[i], r_com)),
        ec_vec3_cross(omega[i], ec_vec3_cross(omega[i], r_com)));
    EC_Vec3 own_force = ec_vec3_scale(com_accel, dynamic->mass);
    EC_Mat3 inertia_world = ec_mat3_mul(
        ec_mat3_mul(rotation[i], dynamic->inertia),
        ec_mat3_transpose(rotation[i]));
    EC_Vec3 own_moment = ec_vec3_add(
        ec_mat3_vec(inertia_world, alpha[i]),
        ec_vec3_cross(omega[i], ec_mat3_vec(inertia_world, omega[i])));

    force[i] = own_force;
    moment[i] = ec_vec3_add(own_moment, ec_vec3_cross(r_com, own_force));
    if (i + 1u < serial->count) {
      EC_Vec3 child_offset = ec_vec3_sub(body_origin[i + 1u], body_origin[i]);
      force[i] = ec_vec3_add(force[i], force[i + 1u]);
      moment[i] = ec_vec3_add(
          moment[i], ec_vec3_add(moment[i + 1u],
                                  ec_vec3_cross(child_offset, force[i + 1u])));
    }
    torque[i] = ec_vec3_dot(
        axis[i], ec_vec3_add(
                     moment[i],
                     ec_vec3_cross(ec_vec3_sub(body_origin[i], joint_origin[i]),
                                   force[i])));
  }
  return true;
}
