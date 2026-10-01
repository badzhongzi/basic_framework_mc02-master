#include "ec_planner.h"

#include <math.h>
#include <string.h>

static float quintic(float t) { t = ec_limit(t,0,1); return t*t*t*(10.0f + t*(-15.0f + 6.0f*t)); }
static EC_Quat quat_slerp(EC_Quat a, EC_Quat b, float t) {
  float dot, angle;
  a = ec_quat_normalize(a);
  b = ec_quat_normalize(b);
  dot=a.v[0]*b.v[0]+a.v[1]*b.v[1]+a.v[2]*b.v[2]+a.v[3]*b.v[3];
  if(dot<0){int i;dot=-dot;for(i=0;i<4;++i)b.v[i]=-b.v[i];}
  dot=ec_limit(dot,-1,1); angle=acosf(dot);
  if(fabsf(angle)<1e-4f){EC_Quat q;int i;for(i=0;i<4;++i)q.v[i]=a.v[i]*(1-t)+b.v[i]*t;return ec_quat_normalize(q);}
  {float s=sinf(angle); EC_Quat q; int i; for(i=0;i<4;++i)q.v[i]=(a.v[i]*sinf((1-t)*angle)+b.v[i]*sinf(t*angle))/s;return ec_quat_normalize(q);}
}
void ec_linear_planner_init(EC_LinearPlanner *p,float period){if(!p)return;memset(p,0,sizeof(*p));p->ctrl_period=period>0?period:0.001f;p->finished=true;}
void ec_linear_planner_set(EC_LinearPlanner *p,float start,float end,float time){if(!p)return;p->start=start;p->end=end;p->plan_time=time>0?time:0;p->elapsed=0;p->finished=p->plan_time<=0;}
float ec_linear_planner_plan(EC_LinearPlanner *p){float t,out;if(!p)return 0;if(p->plan_time<=0){p->finished=true;return p->end;}t=ec_limit(p->elapsed/p->plan_time,0,1);out=p->start+(p->end-p->start)*t;if(p->elapsed>=p->plan_time)p->finished=true;else p->elapsed=fminf(p->elapsed+p->ctrl_period,p->plan_time);return out;}
bool ec_linear_planner_finished(const EC_LinearPlanner *p){return p?p->finished:true;}

void ec_cartesian_planner_init(EC_CartesianPlanner *p,float period){if(!p)return;memset(p,0,sizeof(*p));p->ctrl_period=period>0?period:0.001f;p->finished=true;p->type=EC_CARTESIAN_LINEAR;}
void ec_cartesian_planner_set_linear(EC_CartesianPlanner *p,EC_CartesianCmd start,EC_CartesianCmd end,float time){if(!p)return;p->start=start;p->end=end;p->plan_time=time>0?time:0;p->elapsed=0;p->type=EC_CARTESIAN_LINEAR;p->finished=p->plan_time<=0;}
void ec_cartesian_planner_set_circle(EC_CartesianPlanner *p,EC_CartesianCmd start,EC_CartesianCmd end,EC_Vec3 center,float time,bool major){if(!p)return;p->start=start;p->end=end;p->center=center;p->major_arc=major;p->plan_time=time>0?time:0;p->elapsed=0;p->type=EC_CARTESIAN_CIRCLE;p->finished=p->plan_time<=0;}
EC_CartesianCmd ec_cartesian_planner_plan(EC_CartesianPlanner *p){EC_CartesianCmd out;float u; if(!p){memset(&out,0,sizeof(out));return out;}u=p->plan_time>0?ec_limit(p->elapsed/p->plan_time,0,1):1;
  out=p->start;
  if(p->type==EC_CARTESIAN_LINEAR){EC_Vec3 d=ec_vec3_sub(p->end.pos,p->start.pos);float s=quintic(u);out.pos=ec_vec3_add(p->start.pos,ec_vec3_scale(d,s));}
  else {EC_Vec3 v1=ec_vec3_sub(p->start.pos,p->center),v3=ec_vec3_sub(p->end.pos,p->center),n=ec_vec3_cross(v1,v3);float r=ec_vec3_norm(v1),r3=ec_vec3_norm(v3),n_norm=ec_vec3_norm(n);if(r<1.0e-8f||r3<1.0e-8f||n_norm<1.0e-8f||fabsf(r-r3)>1.0e-4f*(1.0f+fmaxf(r,r3))){EC_Vec3 d=ec_vec3_sub(p->end.pos,p->start.pos);out.pos=ec_vec3_add(p->start.pos,ec_vec3_scale(d,u));}else{EC_Vec3 e1=ec_vec3_scale(v1,1.0f/r),e2=ec_vec3_normalize(ec_vec3_cross(n,e1));float t1=atan2f(ec_vec3_dot(v1,e2),ec_vec3_dot(v1,e1)),t3=atan2f(ec_vec3_dot(v3,e2),ec_vec3_dot(v3,e1)),delta=fmodf(t3-t1+2*EC_PI,2*EC_PI);if(p->major_arc){if(delta<=EC_PI)delta-=2*EC_PI;}else if(delta>EC_PI)delta-=2*EC_PI;{float angle=t1+delta*u;out.pos=ec_vec3_add(p->center,ec_vec3_scale(ec_vec3_add(ec_vec3_scale(e1,cosf(angle)),ec_vec3_scale(e2,sinf(angle))),r));}}}
  out.quat=quat_slerp(p->start.quat,p->end.quat,u);if(p->plan_time<=0||p->elapsed>=p->plan_time)p->finished=true;else p->elapsed=fminf(p->elapsed+p->ctrl_period,p->plan_time);return out;}
bool ec_cartesian_planner_finished(const EC_CartesianPlanner *p){return p?p->finished:true;}

void ec_joint_planner_init(EC_JointPlanner *p,float period,uint8_t count){if(!p)return;memset(p,0,sizeof(*p));p->ctrl_period=period>0?period:0.001f;p->joint_count=count>EC_MAX_JOINTS?EC_MAX_JOINTS:count;p->finished=true;}
void ec_joint_planner_set_periodic(EC_JointPlanner *p,uint8_t index,bool periodic){if(p&&index<p->joint_count)p->periodic[index]=periodic;}
bool ec_joint_planner_set(EC_JointPlanner *p,const float *start,const float *end,float time){uint8_t i;if(!p||!start||!end||time<0)return false;for(i=0;i<p->joint_count;++i){p->start[i]=start[i];p->end[i]=end[i];if(p->periodic[i]){float d=p->end[i]-p->start[i];if(d>EC_PI)p->end[i]-=2*EC_PI;else if(d<-EC_PI)p->end[i]+=2*EC_PI;}}p->plan_time=time;p->elapsed=0;p->finished=time<=0;return true;}
static void quintic_derivatives(float u, float *s1, float *s2) {
  if (s1) *s1 = 30.0f * u * u * (1.0f - u) * (1.0f - u);
  if (s2) *s2 = 60.0f * u * (1.0f - u) * (1.0f - 2.0f * u);
}

void ec_joint_planner_plan_state(EC_JointPlanner *p, float *position,
                                 float *velocity, float *acceleration,
                                 size_t count) {
  uint8_t i;
  float u;
  float s1 = 0.0f;
  float s2 = 0.0f;
  float inv_time = 0.0f;
  float inv_time2 = 0.0f;

  if (!p || !position || count < p->joint_count) return;
  if (p->plan_time > 0.0f) {
    u = ec_limit(p->elapsed / p->plan_time, 0.0f, 1.0f);
    inv_time = 1.0f / p->plan_time;
    inv_time2 = inv_time * inv_time;
    quintic_derivatives(u, &s1, &s2);
  } else {
    u = 1.0f;
  }

  for (i = 0; i < p->joint_count; ++i) {
    float delta = p->end[i] - p->start[i];
    position[i] = p->start[i] + delta * quintic(u);
    if (position[i] > EC_PI) position[i] -= 2.0f * EC_PI;
    else if (position[i] < -EC_PI) position[i] += 2.0f * EC_PI;
    if (velocity) velocity[i] = delta * s1 * inv_time;
    if (acceleration) acceleration[i] = delta * s2 * inv_time2;
  }

  if (p->plan_time <= 0.0f || p->elapsed >= p->plan_time) {
    p->finished = true;
  } else {
    p->elapsed = fminf(p->elapsed + p->ctrl_period, p->plan_time);
  }
}

void ec_joint_planner_plan(EC_JointPlanner *p,float *out,size_t count){
  ec_joint_planner_plan_state(p, out, NULL, NULL, count);
}
bool ec_joint_planner_finished(const EC_JointPlanner *p){return p?p->finished:true;}
