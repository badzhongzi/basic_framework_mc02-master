#include "ec_operation.h"
#include "ec_transform.h"

EC_Vec3 ec_cross(EC_Vec3 a, EC_Vec3 b) { return ec_vec3_cross(a,b); }
float ec_dot(EC_Vec3 a, EC_Vec3 b) { return ec_vec3_dot(a,b); }
EC_Quat ec_quat_cross(EC_Quat a, EC_Quat b) { return ec_quat_mul(a,b); }
EC_Mat4 ec_htm_multiply(EC_Mat4 a, EC_Mat4 b) {
  EC_Mat3 r=ec_mat3_mul(ec_t_to_r(a),ec_t_to_r(b));
  EC_Vec3 p=ec_vec3_add(ec_mat3_vec(ec_t_to_r(a),ec_t_to_p(b)),ec_t_to_p(a));
  return ec_mat4_from_rp(r,p);
}
