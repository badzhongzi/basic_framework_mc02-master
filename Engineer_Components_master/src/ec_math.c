#include "ec_math.h"

#include <math.h>
#include <string.h>

float ec_limit(float value, float min_value, float max_value) {
  return value < min_value ? min_value : (value > max_value ? max_value : value);
}

float ec_norm_period(float lower, float upper, float value) {
  float period = upper - lower;
  float result;
  if (period <= 0.0f) return value;
  result = fmodf(value - lower, period);
  if (result < 0.0f) result += period;
  return result + lower;
}

float ec_norm_angle(float angle) { return ec_norm_period(-EC_PI, EC_PI, angle); }

bool ec_float_equal(float a, float b, float epsilon) {
  return fabsf(a - b) <= (epsilon > 0.0f ? epsilon : 1.0e-4f);
}

bool ec_sincos_bound(float *value, float epsilon) {
  float bound;
  if (!value) return false;
  bound = 1.0f + (epsilon > 0.0f ? epsilon : 0.01f);
  if (fabsf(*value) > bound) return false;
  if (*value > 1.0f) *value = 1.0f;
  if (*value < -1.0f) *value = -1.0f;
  return true;
}

EC_Vec3 ec_vec3(float x, float y, float z) {
  EC_Vec3 out = {{x, y, z}};
  return out;
}
EC_Vec3 ec_vec3_add(EC_Vec3 a, EC_Vec3 b) {
  return ec_vec3(a.v[0] + b.v[0], a.v[1] + b.v[1], a.v[2] + b.v[2]);
}
EC_Vec3 ec_vec3_sub(EC_Vec3 a, EC_Vec3 b) {
  return ec_vec3(a.v[0] - b.v[0], a.v[1] - b.v[1], a.v[2] - b.v[2]);
}
EC_Vec3 ec_vec3_scale(EC_Vec3 a, float scalar) {
  return ec_vec3(a.v[0] * scalar, a.v[1] * scalar, a.v[2] * scalar);
}
float ec_vec3_dot(EC_Vec3 a, EC_Vec3 b) {
  return a.v[0] * b.v[0] + a.v[1] * b.v[1] + a.v[2] * b.v[2];
}
EC_Vec3 ec_vec3_cross(EC_Vec3 a, EC_Vec3 b) {
  return ec_vec3(a.v[1] * b.v[2] - a.v[2] * b.v[1],
                 a.v[2] * b.v[0] - a.v[0] * b.v[2],
                 a.v[0] * b.v[1] - a.v[1] * b.v[0]);
}
float ec_vec3_norm(EC_Vec3 a) { return sqrtf(ec_vec3_dot(a, a)); }
EC_Vec3 ec_vec3_normalize(EC_Vec3 a) {
  float n = ec_vec3_norm(a);
  return n > 1.0e-8f ? ec_vec3_scale(a, 1.0f / n) : ec_vec3(0, 0, 0);
}

void ec_mat3_zero(EC_Mat3 *out) { if (out) memset(out, 0, sizeof(*out)); }
void ec_mat3_identity(EC_Mat3 *out) {
  ec_mat3_zero(out);
  if (out) { out->v[0] = 1.0f; out->v[4] = 1.0f; out->v[8] = 1.0f; }
}
EC_Mat3 ec_mat3_mul(EC_Mat3 a, EC_Mat3 b) {
  EC_Mat3 out;
  int i, j, k;
  ec_mat3_zero(&out);
  for (i = 0; i < 3; ++i) for (j = 0; j < 3; ++j)
    for (k = 0; k < 3; ++k) out.v[i * 3 + j] += a.v[i * 3 + k] * b.v[k * 3 + j];
  return out;
}
EC_Vec3 ec_mat3_vec(EC_Mat3 a, EC_Vec3 b) {
  return ec_vec3(a.v[0] * b.v[0] + a.v[1] * b.v[1] + a.v[2] * b.v[2],
                 a.v[3] * b.v[0] + a.v[4] * b.v[1] + a.v[5] * b.v[2],
                 a.v[6] * b.v[0] + a.v[7] * b.v[1] + a.v[8] * b.v[2]);
}
EC_Mat3 ec_mat3_transpose(EC_Mat3 a) {
  EC_Mat3 out;
  int i, j;
  for (i = 0; i < 3; ++i) for (j = 0; j < 3; ++j) out.v[i * 3 + j] = a.v[j * 3 + i];
  return out;
}
EC_Mat3 ec_hat(EC_Vec3 v) {
  EC_Mat3 out = {{0, -v.v[2], v.v[1], v.v[2], 0, -v.v[0], -v.v[1], v.v[0], 0}};
  return out;
}

void ec_mat4_zero(EC_Mat4 *out) { if (out) memset(out, 0, sizeof(*out)); }
void ec_mat4_identity(EC_Mat4 *out) {
  ec_mat4_zero(out);
  if (out) { out->v[0] = 1.0f; out->v[5] = 1.0f; out->v[10] = 1.0f; out->v[15] = 1.0f; }
}
EC_Mat4 ec_mat4_mul(EC_Mat4 a, EC_Mat4 b) {
  EC_Mat4 out;
  int i, j, k;
  ec_mat4_zero(&out);
  for (i = 0; i < 4; ++i) for (j = 0; j < 4; ++j)
    for (k = 0; k < 4; ++k) out.v[i * 4 + j] += a.v[i * 4 + k] * b.v[k * 4 + j];
  return out;
}
EC_Mat4 ec_mat4_from_rp(EC_Mat3 r, EC_Vec3 p) {
  EC_Mat4 out;
  ec_mat4_zero(&out);
  out.v[0] = r.v[0]; out.v[1] = r.v[1]; out.v[2] = r.v[2]; out.v[3] = p.v[0];
  out.v[4] = r.v[3]; out.v[5] = r.v[4]; out.v[6] = r.v[5]; out.v[7] = p.v[1];
  out.v[8] = r.v[6]; out.v[9] = r.v[7]; out.v[10] = r.v[8]; out.v[11] = p.v[2];
  out.v[15] = 1.0f;
  return out;
}
EC_Mat4 ec_mat4_inverse(EC_Mat4 t) {
  EC_Mat3 r = {{t.v[0], t.v[1], t.v[2], t.v[4], t.v[5], t.v[6], t.v[8], t.v[9], t.v[10]}};
  EC_Mat3 rt = ec_mat3_transpose(r);
  EC_Vec3 p = {{t.v[3], t.v[7], t.v[11]}};
  return ec_mat4_from_rp(rt, ec_vec3_scale(ec_mat3_vec(rt, p), -1.0f));
}

EC_Quat ec_quat(float w, float x, float y, float z) { EC_Quat q = {{w, x, y, z}}; return q; }
EC_Quat ec_quat_identity(void) { return ec_quat(1, 0, 0, 0); }
float ec_quat_norm(EC_Quat q) { return sqrtf(q.v[0]*q.v[0] + q.v[1]*q.v[1] + q.v[2]*q.v[2] + q.v[3]*q.v[3]); }
EC_Quat ec_quat_normalize(EC_Quat q) {
  float n = ec_quat_norm(q);
  return n > 1.0e-8f ? ec_quat(q.v[0]/n, q.v[1]/n, q.v[2]/n, q.v[3]/n) : ec_quat_identity();
}
EC_Quat ec_quat_mul(EC_Quat a, EC_Quat b) {
  return ec_quat(a.v[0]*b.v[0]-a.v[1]*b.v[1]-a.v[2]*b.v[2]-a.v[3]*b.v[3],
                 a.v[0]*b.v[1]+a.v[1]*b.v[0]+a.v[2]*b.v[3]-a.v[3]*b.v[2],
                 a.v[0]*b.v[2]-a.v[1]*b.v[3]+a.v[2]*b.v[0]+a.v[3]*b.v[1],
                 a.v[0]*b.v[3]+a.v[1]*b.v[2]-a.v[2]*b.v[1]+a.v[3]*b.v[0]);
}
EC_Quat ec_quat_conjugate(EC_Quat q) { return ec_quat(q.v[0], -q.v[1], -q.v[2], -q.v[3]); }
EC_Quat ec_quat_inverse(EC_Quat q) {
  float n2 = q.v[0]*q.v[0] + q.v[1]*q.v[1] + q.v[2]*q.v[2] + q.v[3]*q.v[3];
  return n2 > 1.0e-8f ? ec_quat(q.v[0]/n2, -q.v[1]/n2, -q.v[2]/n2, -q.v[3]/n2) : ec_quat_identity();
}
EC_Mat3 ec_quat_to_mat3(EC_Quat q) {
  EC_Mat3 r;
  q = ec_quat_normalize(q);
  { float w=q.v[0], x=q.v[1], y=q.v[2], z=q.v[3];
    r.v[0]=1-2*(y*y+z*z); r.v[1]=2*(x*y-z*w); r.v[2]=2*(x*z+y*w);
    r.v[3]=2*(x*y+z*w); r.v[4]=1-2*(x*x+z*z); r.v[5]=2*(y*z-x*w);
    r.v[6]=2*(x*z-y*w); r.v[7]=2*(y*z+x*w); r.v[8]=1-2*(x*x+y*y); }
  return r;
}
EC_Quat ec_mat3_to_quat(EC_Mat3 r) {
  EC_Quat q; float trace = r.v[0] + r.v[4] + r.v[8]; float s;
  if (trace > 0.0f) { s=sqrtf(trace+1.0f)*2; q=ec_quat(0.25f*s,(r.v[7]-r.v[5])/s,(r.v[2]-r.v[6])/s,(r.v[3]-r.v[1])/s); }
  else if (r.v[0] > r.v[4] && r.v[0] > r.v[8]) { s=sqrtf(1+r.v[0]-r.v[4]-r.v[8])*2; q=ec_quat((r.v[7]-r.v[5])/s,0.25f*s,(r.v[1]+r.v[3])/s,(r.v[2]+r.v[6])/s); }
  else if (r.v[4] > r.v[8]) { s=sqrtf(1+r.v[4]-r.v[0]-r.v[8])*2; q=ec_quat((r.v[2]-r.v[6])/s,(r.v[1]+r.v[3])/s,0.25f*s,(r.v[5]+r.v[7])/s); }
  else { s=sqrtf(1+r.v[8]-r.v[0]-r.v[4])*2; q=ec_quat((r.v[3]-r.v[1])/s,(r.v[2]+r.v[6])/s,(r.v[5]+r.v[7])/s,0.25f*s); }
  return ec_quat_normalize(q);
}
