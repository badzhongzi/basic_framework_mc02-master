#include "ec_transform.h"

#include <math.h>

static float clamp_unit(float x) { return x < -1.0f ? -1.0f : (x > 1.0f ? 1.0f : x); }

EC_Vec3 ec_r_to_rpy(EC_Mat3 r) {
  /* Z-Y-X convention, returned as [yaw, pitch, roll]. */
  EC_Vec3 out;
  float pitch = asinf(clamp_unit(-r.v[6]));
  out.v[0] = atan2f(r.v[3], r.v[0]);
  out.v[1] = pitch;
  out.v[2] = atan2f(r.v[7], r.v[8]);
  if (fabsf(cosf(pitch)) < 1.0e-5f) { out.v[0] = atan2f(-r.v[1], r.v[4]); out.v[2] = 0.0f; }
  return out;
}

EC_Mat3 ec_rpy_to_r(EC_Vec3 rpy) {
  float cy=cosf(rpy.v[0]), sy=sinf(rpy.v[0]), cp=cosf(rpy.v[1]), sp=sinf(rpy.v[1]), cr=cosf(rpy.v[2]), sr=sinf(rpy.v[2]);
  EC_Mat3 r = {{cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr,
                sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr,
                -sp, cp*sr, cp*cr}};
  return r;
}

void ec_r_to_angvec4(EC_Mat3 r, float out[4]) {
  float c = clamp_unit((r.v[0]+r.v[4]+r.v[8]-1.0f)*0.5f);
  float skew_x = r.v[7] - r.v[5];
  float skew_y = r.v[2] - r.v[6];
  float skew_z = r.v[3] - r.v[1];
  float sin_angle = 0.5f * sqrtf(skew_x * skew_x + skew_y * skew_y +
                                 skew_z * skew_z);
  float angle = atan2f(sin_angle, c);
  float s;
  if (angle < 1.0e-5f) { out[0]=1; out[1]=0; out[2]=0; out[3]=0; return; }
  s = 2.0f*sinf(angle);
  if (c < 0.0f && sin_angle < 1.0e-5f) {
    float x2 = fmaxf(0.0f, (r.v[0] + 1.0f) * 0.5f);
    float y2 = fmaxf(0.0f, (r.v[4] + 1.0f) * 0.5f);
    float z2 = fmaxf(0.0f, (r.v[8] + 1.0f) * 0.5f);
    if (x2 >= y2 && x2 >= z2 && x2 > 1.0e-8f) {
      out[0] = sqrtf(x2);
      out[1] = (r.v[1] + r.v[3]) / (4.0f * out[0]);
      out[2] = (r.v[2] + r.v[6]) / (4.0f * out[0]);
    } else if (y2 >= z2 && y2 > 1.0e-8f) {
      out[1] = sqrtf(y2);
      out[0] = (r.v[1] + r.v[3]) / (4.0f * out[1]);
      out[2] = (r.v[5] + r.v[7]) / (4.0f * out[1]);
    } else if (z2 > 1.0e-8f) {
      out[2] = sqrtf(z2);
      out[0] = (r.v[2] + r.v[6]) / (4.0f * out[2]);
      out[1] = (r.v[5] + r.v[7]) / (4.0f * out[2]);
    } else {
      out[0] = 1.0f; out[1] = 0.0f; out[2] = 0.0f;
    }
    out[3] = angle;
    { EC_Vec3 axis = ec_vec3_normalize(ec_vec3(out[0], out[1], out[2]));
      out[0] = axis.v[0]; out[1] = axis.v[1]; out[2] = axis.v[2]; }
    return;
  }
  out[0]=skew_x/s; out[1]=skew_y/s; out[2]=skew_z/s; out[3]=angle;
}
EC_Vec3 ec_r_to_angvec(EC_Mat3 r) { float a[4]; ec_r_to_angvec4(r,a); return ec_vec3(a[0],a[1],a[2]); }
EC_Mat3 ec_angvec_to_r(const float a[4]) {
  EC_Vec3 axis=ec_vec3(a[0],a[1],a[2]); float angle=a[3], c=cosf(angle), s=sinf(angle), k=1-c;
  axis=ec_vec3_normalize(axis); {
    float x=axis.v[0],y=axis.v[1],z=axis.v[2];
    EC_Mat3 r={{c+x*x*k,x*y*k-z*s,x*z*k+y*s,y*x*k+z*s,c+y*y*k,y*z*k-x*s,z*x*k-y*s,z*y*k+x*s,c+z*z*k}}; return r;
  }
}
EC_Vec3 ec_quat_to_rpy(EC_Quat q) { return ec_r_to_rpy(ec_quat_to_mat3(q)); }
EC_Quat ec_rpy_to_quat(EC_Vec3 rpy) { return ec_mat3_to_quat(ec_rpy_to_r(rpy)); }
void ec_quat_to_angvec(EC_Quat q, float out[4]) { ec_r_to_angvec4(ec_quat_to_mat3(q), out); }
EC_Quat ec_angvec_to_quat(const float a[4]) { return ec_mat3_to_quat(ec_angvec_to_r(a)); }

EC_Mat3 ec_t_to_r(EC_Mat4 t) { EC_Mat3 r={{t.v[0],t.v[1],t.v[2],t.v[4],t.v[5],t.v[6],t.v[8],t.v[9],t.v[10]}}; return r; }
EC_Vec3 ec_t_to_p(EC_Mat4 t) { return ec_vec3(t.v[3],t.v[7],t.v[11]); }
EC_Mat4 ec_r_to_t(EC_Mat3 r) { return ec_mat4_from_rp(r,ec_vec3(0,0,0)); }
EC_Mat4 ec_p_to_t(EC_Vec3 p) { EC_Mat3 r; ec_mat3_identity(&r); return ec_mat4_from_rp(r,p); }
EC_Mat4 ec_rp_to_t(EC_Mat3 r, EC_Vec3 p) { return ec_mat4_from_rp(r,p); }
EC_Mat4 ec_t_inverse(EC_Mat4 t) { return ec_mat4_inverse(t); }
EC_Vec3 ec_t_to_rpy(EC_Mat4 t) { return ec_r_to_rpy(ec_t_to_r(t)); }
EC_Mat4 ec_rpy_to_t(EC_Vec3 rpy) { return ec_r_to_t(ec_rpy_to_r(rpy)); }
EC_Quat ec_t_to_quat(EC_Mat4 t) { return ec_mat3_to_quat(ec_t_to_r(t)); }
EC_Mat4 ec_quat_to_t(EC_Quat q) { return ec_r_to_t(ec_quat_to_mat3(q)); }
void ec_t_to_twist(EC_Mat4 t, float out[6]) { float a[4]; EC_Vec3 p=ec_t_to_p(t); ec_r_to_angvec4(ec_t_to_r(t),a); out[0]=p.v[0];out[1]=p.v[1];out[2]=p.v[2];out[3]=a[0]*a[3];out[4]=a[1]*a[3];out[5]=a[2]*a[3]; }
EC_Mat4 ec_twist_to_t(const float x[6]) { float angle=sqrtf(x[3]*x[3]+x[4]*x[4]+x[5]*x[5]), a[4]; a[0]=angle>1e-8f?x[3]/angle:1; a[1]=angle>1e-8f?x[4]/angle:0; a[2]=angle>1e-8f?x[5]/angle:0; a[3]=angle; return ec_rp_to_t(ec_angvec_to_r(a),ec_vec3(x[0],x[1],x[2])); }
/* Same MDH convention and matrix formula as the original C++ implementation. */
EC_Mat4 ec_mdh_to_transform(float theta, float d, float a, float alpha) { float ct=cosf(theta),st=sinf(theta),ca=cosf(alpha),sa=sinf(alpha); EC_Mat4 t={{ct,-st,0,a,st*ca,ct*ca,-sa,-sa*d,st*sa,ct*sa,ca,ca*d,0,0,0,1}}; return t; }
