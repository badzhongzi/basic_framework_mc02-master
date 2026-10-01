#include "ec_ikine.h"

#include "ec_math.h"
#include "ec_transform.h"

#include <math.h>
#include <string.h>

static void clear_solution(float *solution, size_t count) { if(solution) memset(solution,0,sizeof(float)*count); }

/* Analytic 6-DOF solver corresponding to the original ikine5 model. */
uint8_t ec_ikine5(EC_ModuleCartesianCmd cmd,const float l[3],float solution[8][6]) {
  EC_Mat4 t, t1, t2, t3, t123, pos;
  float x,y,z,nx,ny,nz,xs,ys,zs;
  float tmp_theta1,tmp_c1; uint8_t i,num=0;
  if (!l || !solution) return 0;
  clear_solution(&solution[0][0],8*6);
  t=ec_quat_to_t(cmd.quat);
  t.v[3]=cmd.pos.v[0];
  t.v[7]=cmd.pos.v[1];
  t.v[11]=cmd.pos.v[2];
  x=t.v[3];
  y=t.v[7];
  z=t.v[11];
  nx=l[2]*t.v[2];
  ny=l[2]*t.v[6];
  nz=l[2]*t.v[10];
  xs=x-nx;
  ys=y-ny;
  zs=z-nz;
  tmp_theta1=atan2f(ys,xs);
  {
    float d=sqrtf(xs*xs+ys*ys);
    tmp_c1=d>1e-8f?xs/d:1;
  }
  for(i=0;i<8;++i)
  {
    float c1,a,b,m,n,c2,s2,c3,s3,c5,s5;solution[i][0]=(i<4)?tmp_theta1:tmp_theta1+EC_PI;c1=(i<4)?tmp_c1:-tmp_c1;a=2*l[0]*l[1];b=0;c2=((xs/c1)*(xs/c1)+zs*zs-l[1]*l[1]-l[0]*l[0])/a;if(!ec_sincos_bound(&c2,.01f))continue;s2=(i%4<2)?sqrtf(fmaxf(0,1-c2*c2)):-sqrtf(fmaxf(0,1-c2*c2));solution[i][2]=atan2f(s2,c2);c3=cosf(solution[i][2]);s3=sinf(solution[i][2]);a=l[1]*c3+l[0];b=l[1]*s3;m=xs/c1;n=zs;s2=(a*n-b*m)/(a*a+b*b);c2=(m*a+n*b)/(a*a+b*b);if(fabsf(s2)>1.01f||fabsf(c2)>1.01f)continue;solution[i][1]=atan2f(s2,c2);
    t1=ec_mdh_to_transform(solution[i][0],0,0,0);t2=ec_mdh_to_transform(solution[i][1],0,0,EC_PI/2);t3=ec_mdh_to_transform(solution[i][2]+EC_PI/2,0,l[0],0);t123=ec_mat4_mul(ec_mat4_mul(t1,t2),t3);pos=ec_mat4_mul(ec_mat4_inverse(t123),t);c5=-pos.v[6];if(!ec_sincos_bound(&c5,.01f))continue;
    if(fabsf(c5)>.999f){solution[i][4]=c5>0?0:EC_PI;solution[i][3]=0;solution[i][5]=c5>0?atan2f(pos.v[8],pos.v[0]):-atan2f(pos.v[8],pos.v[0]);}
    else {s5=(i%2==0)?sqrtf(fmaxf(0,1-c5*c5)):-sqrtf(fmaxf(0,1-c5*c5));solution[i][4]=atan2f(s5,c5);solution[i][3]=atan2f(pos.v[10]/(-s5),pos.v[2]/(-s5));solution[i][5]=atan2f(pos.v[5]/(-s5),pos.v[4]/s5);}
    {uint8_t j;for(j=0;j<6;++j)solution[num][j]=ec_norm_angle(solution[i][j]);}++num;
  }
  /* The branch generation above can produce duplicate rows at singularities. */
  return num;
}

uint8_t ec_ikine1(EC_ModuleCartesianCmd cmd,const float l[4],float sol[8][6]) {
  EC_Mat3 rotation;
  EC_Mat4 target;
  EC_Vec3 wrist;
  EC_Vec3 tool;
  uint8_t branch;
  uint8_t count = 0;

  if (!l || !sol || fabsf(l[0]) < 1.0e-8f ||
      fabsf(l[1]) < 1.0e-8f || fabsf(l[3]) < 1.0e-8f) {
    return 0;
  }
  clear_solution(&sol[0][0], 8 * 6);
  rotation = ec_quat_to_mat3(cmd.quat);
  target = ec_mat4_from_rp(rotation, cmd.pos);
  tool = ec_mat3_vec(rotation, ec_vec3(0.0f, 0.0f, l[3]));
  wrist = ec_vec3_sub(cmd.pos, tool);

  for (branch = 0; branch < 8; ++branch) {
    float candidate[6];
    float radial = sqrtf(wrist.v[0] * wrist.v[0] +
                         wrist.v[2] * wrist.v[2]);
    float s1;
    float c1;
    float s5;
    float c5;
    float s234;
    float c234;
    float theta234;
    float a;
    float b;
    float c3;
    float s3;
    float c;
    float d;
    float denominator;
    float s2;
    float c2;
    EC_Mat4 t15;
    EC_Mat4 wrist_transform;
    uint8_t joint;

    if (radial < 1.0e-8f) continue;
    s1 = (branch < 4 ? wrist.v[2] : -wrist.v[2]) / radial;
    c1 = (branch < 4 ? wrist.v[0] : -wrist.v[0]) / radial;
    candidate[0] = atan2f(s1, c1);

    s5 = s1 * rotation.v[2] - c1 * rotation.v[8];
    if (!ec_sincos_bound(&s5, 0.01f)) continue;
    c5 = (branch < 2 || branch > 5)
             ? sqrtf(fmaxf(0.0f, 1.0f - s5 * s5))
             : -sqrtf(fmaxf(0.0f, 1.0f - s5 * s5));
    if (fabsf(c1 * c5) < 1.0e-8f) continue;
    candidate[4] = atan2f(s5, c5);

    s234 = -tool.v[1] / (l[3] * c5);
    c234 = (tool.v[0] / l[3] - s1 * s5) / (c1 * c5);
    if (!isfinite(s234) || !isfinite(c234) ||
        s234 * s234 + c234 * c234 > 1.005f) {
      continue;
    }
    theta234 = atan2f(s234, c234);

    a = wrist.v[0] / c1 - l[2] * c234;
    b = -wrist.v[1] - l[2] * s234;
    c3 = (a * a + b * b - l[0] * l[0] - l[1] * l[1]) /
         (2.0f * l[0] * l[1]);
    if (!ec_sincos_bound(&c3, 0.01f)) continue;
    s3 = branch % 2 == 0 ? sqrtf(fmaxf(0.0f, 1.0f - c3 * c3))
                          : -sqrtf(fmaxf(0.0f, 1.0f - c3 * c3));
    candidate[2] = atan2f(s3, c3);

    c = l[0] + l[1] * c3;
    d = l[1] * s3;
    denominator = d * d + c * c;
    if (denominator < 1.0e-12f) continue;
    s2 = (b * c - a * d) / denominator;
    c2 = (b * d + a * c) / denominator;
    if (!isfinite(s2) || !isfinite(c2) ||
        s2 * s2 + c2 * c2 > 1.01f) {
      continue;
    }
    candidate[1] = atan2f(s2, c2);
    candidate[3] = theta234 - candidate[1] - candidate[2];

    t15 = ec_mdh_to_transform(candidate[0], 0.0f, 0.0f, EC_PI / 2.0f);
    t15 = ec_mat4_mul(
        t15, ec_mdh_to_transform(candidate[1], 0.0f, 0.0f,
                                 EC_PI / 2.0f));
    t15 = ec_mat4_mul(
        t15, ec_mdh_to_transform(candidate[2], 0.0f, l[0], 0.0f));
    t15 = ec_mat4_mul(
        t15, ec_mdh_to_transform(candidate[3], 0.0f, l[1], 0.0f));
    t15 = ec_mat4_mul(
        t15, ec_mdh_to_transform(candidate[4] + EC_PI / 2.0f, 0.0f,
                                 l[2], EC_PI / 2.0f));
    wrist_transform = ec_mat4_mul(ec_mat4_inverse(t15), target);
    candidate[5] = atan2f(wrist_transform.v[8], wrist_transform.v[9]);

    for (joint = 0; joint < 6; ++joint) {
      sol[count][joint] = ec_norm_angle(candidate[joint]);
    }
    ++count;
  }
  return count;
}

uint8_t ec_ikine2(EC_ModuleCartesianCmd cmd,const float l[4],float sol[8][6]) {
  EC_Mat4 t; EC_Mat3 r; float x,y,z,nx,ny,nz,xs,ys,zs; uint8_t i,num=0; if(!l||!sol)return 0;clear_solution(&sol[0][0],8*6);r=ec_quat_to_mat3(cmd.quat);t=ec_mat4_from_rp(r,cmd.pos);x=t.v[3];y=t.v[7];z=t.v[11];nx=l[3]*t.v[2];ny=l[3]*t.v[6];nz=l[3]*t.v[10];xs=x-nx;ys=y-ny;zs=z-nz;
  {float angle=-65.0f/180.0f*EC_PI,ca=cosf(angle),sa=sinf(angle);for(i=0;i<8;++i){float den=sqrtf(xs*xs+ys*ys),c1,s1,a,b,c,ct3,st3,th3,c3,s3,c3a,s3a,m,n,s2,c2,c23,s23,lx,ly,lz,ct5,st5,th5,s5,c4,s4;bool ok=true; c1=(i<4?xs:-xs)/(den>1e-8f?den:1);s1=(i<4?ys:-ys)/(den>1e-8f?den:1);sol[i][0]=atan2f(s1,c1);a=2*l[0]*l[1]-2*l[0]*l[2]*sa;b=2*l[0]*l[2]*ca;c=(xs/c1)*(xs/c1)+zs*zs-l[1]*l[1]+2*l[1]*l[2]*sa-l[2]*l[2]-l[0]*l[0];ct3=c/sqrtf(a*a+b*b);if(!ec_sincos_bound(&ct3,.01f))ok=false;st3=sqrtf(fmaxf(0,1-ct3*ct3));th3=atan2f(st3,ct3)-atan2f(b,a);if(!(i%4==0||i%4==1))th3=-atan2f(st3,ct3)-atan2f(b,a);sol[i][2]=th3;c3=cosf(th3);s3=sinf(th3);c3a=c3*ca-s3*sa;s3a=c3*sa+s3*ca;a=l[1]*c3+l[0]-l[2]*s3a;b=l[1]*s3+l[2]*c3a;m=xs/c1;n=zs;s2=(a*n-b*m)/(a*a+b*b);c2=(m*a+n*b)/(a*a+b*b);if(fabsf(c2)>1.01f||fabsf(s2)>1.01f)ok=false;sol[i][1]=atan2f(a*n-b*m,a*m+b*n);c23=c2*c3a-s2*s3a;s23=c2*s3a+c3a*s2;lx=-l[2]*s23*c1;ly=-l[2]*s23*s1;lz=l[2]*c23;ct5=(lz*nz+ly*ny+lx*nx)/(l[2]*l[3]);if(!ec_sincos_bound(&ct5,.01f))ok=false;st5=sqrtf(fmaxf(0,1-ct5*ct5));if(fabsf(st5)>1.01f)ok=false;th5=atan2f(st5,ct5);s5=(i%2==0)?st5:-st5;sol[i][4]=(i%2==0)?th5:-th5;c4=(t.v[10]-c23*ct5)/(s5*s23);s4=(((t.v[6]+s23*ct5*s1)/s5)-c23*c4*s1)/c1;if(fabsf(c4)>1.01f||fabsf(s4)>1.01f)ok=false;sol[i][3]=atan2f(s4,c4);a=s23*c1*s5-ct5*s1*s4+c23*c1*c4*ct5;b=c4*s1+c23*c1*s4;c=-(c23*s5-s23*c4*ct5);{float d=-s23*s4;float den2=a*d+c*b;if(den2<.001f){if(t.v[0]*d+t.v[8]*b>.001f||a*t.v[8]-t.v[0]*c>.001f)ok=false;sol[i][5]=0;}else{float c6=(t.v[0]*d+t.v[8]*b)/den2,s6=(a*t.v[8]-t.v[0]*c)/(a*d+b*c);if(fabsf(c6)>1.01f||fabsf(s6)>1.01f)ok=false;sol[i][5]=atan2f(s6,c6);}}if(ok){uint8_t j;for(j=0;j<6;++j)sol[num][j]=ec_norm_angle(sol[i][j]);++num;}}}
  return num;
}

uint8_t ec_ikine3(EC_ModuleCartesianCmd cmd,const float l[4],float theta6,float theta7,float sol[4][7]) {
  EC_Mat4 t,t6,t7,ts;uint8_t i,num=0;if(!l||!sol)return 0;clear_solution(&sol[0][0],4*7);t=ec_quat_to_t(cmd.quat);t.v[3]=cmd.pos.v[0];t.v[7]=cmd.pos.v[1];t.v[11]=cmd.pos.v[2];t6=ec_mdh_to_transform(theta6+EC_PI/2,0,l[2],0);t7=ec_mdh_to_transform(theta7,l[3],0,EC_PI/2);ts=ec_mat4_mul(t,ec_mat4_inverse(ec_mat4_mul(t6,t7)));{float x=ts.v[3],z=ts.v[11],den=sqrtf(x*x+z*z),tmp=atan2f(z,x),c1=x/(den>1e-8f?den:1),s1=z/(den>1e-8f?den:1);for(i=0;i<4;++i){float th1=i<2?tmp:tmp+EC_PI,cc=i<2?c1:-c1,ss=i<2?s1:-s1,a=ts.v[3]/cc,b=-ts.v[7],ct3=(a*a+b*b-l[0]*l[0]-l[1]*l[1])/(2*l[0]*l[1]);if(!ec_sincos_bound(&ct3,.01f))continue;float th3=(i==0||i==3)?acosf(ct3):-acosf(ct3),c3=cosf(th3),s3=sinf(th3),c=l[1]*c3+l[0],d=l[1]*s3,c2=(a*c+b*d)/(c*c+d*d),s2=(b*c-a*d)/(c*c+d*d),th2=atan2f(s2,c2),a1=ts.v[0],a2=ts.v[4],a3=ts.v[8],b1=ts.v[1],b2=ts.v[5],b3=ts.v[9],e1=ts.v[2],e2=ts.v[6],e3=ts.v[10],s23=sinf(th2+th3),c23=cosf(th2+th3),sig=e2*s23-e1*c23*cc-e3*c23*ss;if(fabsf(sig)>.01f)continue;float s4=e1*ss-e3*cc,c4=-e2*c23-e1*s23*cc-e3*s23*ss,s5=b2*s23-b1*c23*cc-b3*c23*ss,c5=-(a2*s23-a1*c23*cc-a3*c23*ss);sol[num][0]=th1;sol[num][1]=th2;sol[num][2]=th3;sol[num][3]=atan2f(s4,c4);sol[num][4]=atan2f(s5,c5);sol[num][5]=theta6;sol[num][6]=theta7;++num;}}return num;
}

uint8_t ec_ikine4(EC_ModuleCartesianCmd cmd,const float l[6],float theta7,float sol[8][7]) {
  EC_Mat3 r;EC_Mat4 t,t7,ts,tn1,tn2;float p[3];uint8_t i,num=0;if(!l||!sol)return 0;clear_solution(&sol[0][0],8*7);r=ec_quat_to_mat3(cmd.quat);p[0]=cmd.pos.v[0]-(r.v[0]*l[5]);p[1]=cmd.pos.v[1]-(r.v[3]*l[5]);p[2]=cmd.pos.v[2]-(r.v[6]*l[5]);t=ec_mat4_from_rp(r,ec_vec3(p[0],p[1],p[2]));t7=ec_mdh_to_transform(theta7,0,l[4],0);ts=ec_mat4_mul(t,ec_mat4_inverse(t7));{float th1=atan2f(ts.v[11],ts.v[3]),th1b=th1+EC_PI;EC_Mat4 t1=ec_mdh_to_transform(th1,0,0,EC_PI/2),t1b=ec_mdh_to_transform(th1b,0,0,EC_PI/2);tn1=ec_mat4_mul(ec_mat4_inverse(t1),ts);tn2=ec_mat4_mul(ec_mat4_inverse(t1b),ts);float th6a=atan2f(tn1.v[4],tn1.v[5]),th6b=atan2f(tn2.v[4],tn2.v[5]);for(i=0;i<8;++i){EC_Mat4 *tn=i<4?&tn1:&tn2;float th6=i<4?(i<2?th6a:th6a+EC_PI):(i>5?th6b:th6b+EC_PI),s6=sinf(th6),c6=cosf(th6),c5=fabsf(s6)<.01f?-tn->v[5]/c6:-tn->v[4]/s6,s5=tn->v[6],th5=atan2f(s5,c5),th234=atan2f(tn->v[2]/c5,-tn->v[10]/c5),c234=cosf(th234),s234=sinf(th234),a=tn->v[3]-l[3]*c234-l[0],b=tn->v[11]-l[3]*s234,c3=(a*a+b*b-l[1]*l[1]-l[2]*l[2])/(2*l[1]*l[2]);if(!ec_sincos_bound(&c3,.05f))continue;float s3=(i%2)?sqrtf(fmaxf(0,1-c3*c3)):-sqrtf(fmaxf(0,1-c3*c3)),th3=atan2f(s3,c3),cc=l[2]*c3+l[1],dd=l[2]*s3,th2=atan2f((b*cc-a*dd)/(cc*cc+dd*dd),(a*cc+b*dd)/(cc*cc+dd*dd));sol[num][0]=i<4?th1:th1b;sol[num][1]=th2;sol[num][2]=th3;sol[num][3]=th234-th2-th3;sol[num][4]=th5;sol[num][5]=th6;sol[num][6]=theta7;++num;}}return num;
}
