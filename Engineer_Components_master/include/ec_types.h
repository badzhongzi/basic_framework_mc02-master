#ifndef ENGINEER_COMPONENTS_C_TYPES_H
#define ENGINEER_COMPONENTS_C_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef EC_PI
#define EC_PI 3.14159265358979323846f
#endif

/* 这些上限保证对象可以静态分配，适合 STM32 等嵌入式目标。 */
#ifndef EC_MAX_JOINTS
#define EC_MAX_JOINTS 8u
#endif
#ifndef EC_MAX_IK_SOLUTIONS
#define EC_MAX_IK_SOLUTIONS 8u
#endif

/* 向量和矩阵均采用行优先存储。 */
typedef struct { float v[3]; } EC_Vec3; /* 三维向量 [x, y, z]；分量单位由具体物理量决定。 */
typedef struct { float v[9]; } EC_Mat3; /* 3x3 矩阵；元素无固定物理单位。 */
typedef struct { float v[16]; } EC_Mat4; /* 4x4 齐次变换矩阵；平移元素使用长度单位，其余元素无量纲。 */
typedef struct { float v[4]; } EC_Quat; /* 四元数 [w, x, y, z]；无量纲。 */

/* 笛卡尔空间命令：末端位置 + 姿态。 */
typedef struct {
  EC_Vec3 pos;  /* 位置，长度单位由应用统一（示例使用 m）。 */
  EC_Quat quat; /* 姿态，四元数顺序为 [w, x, y, z]；姿态角等效单位为 rad。 */
} EC_CartesianCmd;

/* 与模块层命令同布局的笛卡尔命令类型，供逆解回调使用。 */
typedef EC_CartesianCmd EC_ModuleCartesianCmd;

/* MDH（Modified DH）参数，描述单节连杆的几何关系。 */
typedef struct {
  float offset; /* 关节零位偏置，单位 rad */
  float d;      /* MDH 参数 d，长度单位由应用统一（示例使用 m）。 */
  float a;      /* MDH 参数 a，长度单位由应用统一（示例使用 m）。 */
  float alpha;  /* MDH 参数 alpha，单位 rad */
  float qmin;   /* 关节最小角度，单位 rad */
  float qmax;   /* 关节最大角度，单位 rad */
} EC_MDH;

/* 连杆动力学参数，用于重力补偿和动力学计算。 */
typedef struct {
  float mass;          /* 质量，单位 kg */
  EC_Vec3 centroid;    /* 质心在连杆坐标系中的位置，长度单位由应用统一（示例使用 m）。 */
  EC_Mat3 inertia;     /* 惯量矩阵，单位 kg·m²（长度单位为 m 时）。 */
} EC_DynamicParam;

/* 单节连杆：MDH 几何参数 + 动力学参数。 */
typedef struct {
  EC_MDH mdh;                  /* MDH 几何参数；长度单位由应用统一（示例使用 m）。 */
  EC_DynamicParam dynamic;     /* 动力学参数 */
} EC_Link;

/* 串联连杆模型：从基座到末端依次存储各节连杆，并缓存正解结果。 */
typedef struct {
  uint16_t count;                         /* 当前连杆数量 */
  EC_Link links[EC_MAX_JOINTS];            /* 从基座到末端依次存储 */
  EC_Mat4 fkine;                           /* 最近一次正解结果；平移单位同 MDH 长度单位。 */
  EC_Mat4 transforms[EC_MAX_JOINTS];       /* 最近一次各节局部变换；平移单位同 MDH 长度单位。 */
} EC_SerialLink;

/* 标量线性规划器：在起止标量之间做线性插值，每个控制周期推进一次。 */
typedef struct {
  float ctrl_period; /* 控制周期，单位 s */
  float start;       /* 起点；标量物理单位由应用统一。 */
  float end;         /* 终点；标量物理单位由应用统一。 */
  float plan_time;   /* 规划总时间，单位 s */
  float elapsed;     /* 已规划时间，单位 s */
  bool finished;     /* 是否已经到达终点 */
} EC_LinearPlanner;

/* 笛卡尔规划轨迹类型：直线或圆弧。 */
typedef enum {
  EC_CARTESIAN_LINEAR = 0, /* 直线插值：位置沿空间直线，姿态做四元数 SLERP。 */
  EC_CARTESIAN_CIRCLE = 1  /* 圆弧插值：位置沿起点-圆心-终点确定的圆弧，姿态做四元数 SLERP。 */
} EC_CartesianPlanType;

/* 笛卡尔空间规划器：在两个笛卡尔位姿之间做直线或圆弧插值。 */
typedef struct {
  float ctrl_period;             /* 控制周期，单位 s */
  float elapsed;                 /* 已规划时间，单位 s */
  float plan_time;               /* 规划总时间，单位 s */
  bool finished;                 /* 是否已经完成 */
  EC_CartesianPlanType type;     /* 直线或圆弧 */
  EC_CartesianCmd start;         /* 起始位姿；位置使用长度单位，姿态角使用 rad。 */
  EC_CartesianCmd end;           /* 目标位姿；位置使用长度单位，姿态角使用 rad。 */
  EC_Vec3 center;                /* 圆弧圆心，长度单位由应用统一（示例使用 m）；直线规划时未使用。 */
  bool major_arc;                /* 是否选择互补方向的圆弧；对径端点时两条路径均为 180°。 */
} EC_CartesianPlanner;

/* 关节空间规划器：在两组关节角之间做五次多项式插值。 */
typedef struct {
  float ctrl_period;                    /* 控制周期，单位 s */
  uint8_t joint_count;                  /* 实际参与规划的关节数 */
  float start[EC_MAX_JOINTS];           /* 起始关节角，单位 rad */
  float end[EC_MAX_JOINTS];             /* 目标关节角，单位 rad */
  float plan_time;                      /* 规划总时间，单位 s */
  float elapsed;                        /* 已规划时间，单位 s */
  bool finished;                        /* 是否已经完成 */
  bool periodic[EC_MAX_JOINTS];         /* 是否按最短周期角差规划；角度单位 rad。 */
} EC_JointPlanner;

/* 一次规划采样得到的关节状态参考。位置单位 rad，速度单位 rad/s，
 * 加速度单位 rad/s^2。数组前 joint_count 项有效。 */
typedef struct {
  uint8_t joint_count;
  float q[EC_MAX_JOINTS];
  float q_dot[EC_MAX_JOINTS];
  float q_ddot[EC_MAX_JOINTS];
} EC_JointStateReference;

/* 命令所在的工作空间：关节空间或笛卡尔空间。 */
typedef enum {
  EC_WORKSPACE_JOINT = 0,     /* 关节空间：命令直接指定各关节角度。 */
  EC_WORKSPACE_CARTESIAN = 1  /* 笛卡尔空间：命令指定末端位置和姿态。 */
} EC_Workspace;

/* 规划类型：决定 EC_ArmFsmCmd 中路径节点的插值方式。 */
typedef enum {
  EC_PLAN_JOINT = 0,      /* 关节空间插值：各关节独立做五次多项式插值。 */
  EC_PLAN_LINEAR = 1,     /* 笛卡尔直线插值：位置走空间直线，姿态做四元数 SLERP。 */
  EC_PLAN_CIRCULAR = 2,   /* 笛卡尔圆弧插值：位置走圆弧，姿态做四元数 SLERP。 */
  EC_PLAN_NONE = 3        /* 无规划：直接使用节点中的关节角或逆解结果，不做轨迹插值。 */
} EC_PlanType;

/* 单条机械臂命令：指定一个目标位置，可以是关节角或笛卡尔位姿。 */
typedef struct {
  EC_Workspace workspace;               /* 命令所在空间：关节空间或笛卡尔空间。 */
  EC_CartesianCmd cartesian;            /* 笛卡尔命令；workspace == CARTESIAN 时有效。 */
  uint8_t joint_count;                  /* 关节命令数量；workspace == JOINT 时有效。 */
  float joint[EC_MAX_JOINTS];           /* 关节角，单位 rad；workspace == JOINT 时有效。 */
  uint8_t id;                           /* 命令标识，供上层应用区分不同命令。 */
} EC_ArmCmd;

/* 机械臂 FSM 命令：包含多节点路径和规划类型，驱动一次完整运动。 */
typedef struct {
  uint8_t node_count;                   /* 有效路径节点数量，最多 8 个。 */
  EC_ArmCmd nodes[8];                   /* 路径节点；各规划类型对节点的解释见 ec_arm_generate_reference()。 */
  EC_PlanType plan_type;                /* 规划类型：关节插值、直线插值或圆弧插值。 */
  float traj_time[8];                   /* 轨迹时间，单位 s；关节规划时 traj_time[i] 是从前一节点
                                           到 nodes[i] 的时长（i=0 的前一节点是当前反馈角）；
                                           直线/圆弧规划只使用 traj_time[0]。 */
} EC_ArmFsmCmd;

/* 机械臂当前位姿：同时包含笛卡尔位姿、关节角和齐次变换三种表示。 */
typedef struct {
  EC_CartesianCmd cartesian;            /* 当前末端笛卡尔位姿；位置为长度单位，姿态等效角为 rad。 */
  uint8_t joint_count;                  /* 当前关节数量 */
  float joint[EC_MAX_JOINTS];           /* 当前关节角，单位 rad */
  EC_Mat4 matrix;                       /* 当前末端齐次变换；平移单位同 MDH 长度单位。 */
} EC_ArmPose;

#endif
