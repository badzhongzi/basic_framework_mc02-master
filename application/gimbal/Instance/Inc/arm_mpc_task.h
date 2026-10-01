#ifndef ARM_MPC_TASK_H
#define ARM_MPC_TASK_H

#include "ins_arm.h"
#include "ins_traj.h"
#include "dmmotor.h"
#include "cmsis_os.h"

#define ARM_MPC_JOINT_COUNT INS_ARM_6DOF_COUNT

/*
 * 工作模式。由上层每周期通过 ArmMpcTaskSetMode() 给出,
 * 本模块只负责模式之间的安全切换, 不决定"现在该是哪个模式"。
 *
 *   ZERO_FORCE  电机不出力 (MIT 帧 Kp=Kd=tau=0), 目标角每周期被改写成实际角,
 *               所以切回 RUN 时机械臂停在原地, 不会往旧目标跳。
 *               上电默认处于该模式。切到这个模式同时也是故障确认:
 *               清除已锁存的故障标志, 并对报错的电机发清错指令。
 *   RUN         走向目标角, 到了就停在目标角。
 */
typedef enum {
    ARM_MPC_MODE_ZERO_FORCE = 0,
    ARM_MPC_MODE_RUN,
} ArmMpcMode;

/*
 * 走向目标角的方式。
 *
 *   FOLLOW  跟随: 目标可以每周期都变, 每次按固定平滑时长 (100 ms) 追过去。
 *           给遥控器用: 增量式在上层累加成目标角, 绝对式直接给目标角。
 *   TIMED   定时: 目标变化时按 move_time 走过去。给点位/自动流程用。
 *
 * 两种方式都从"此刻的参考状态"起步, 中途换目标、换方式都不会跳变;
 * 按给定时长会超出速度/加速度上限时, 轨迹层会自动把时长拉长。
 */
typedef enum {
    ARM_MPC_MOVE_FOLLOW = 0,
    ARM_MPC_MOVE_TIMED,
} ArmMpcMove;

/* 锁存的故障原因, 按位或。任一位置位都会让输出进入零力矩。 */
#define ARM_MPC_FAULT_MOTOR_OFFLINE 0x01u /* 已配置的电机掉线、从未收到反馈或处于失能状态 */
#define ARM_MPC_FAULT_FEEDBACK      0x02u /* 反馈角/角速度非有限值 */
#define ARM_MPC_FAULT_TRAJ          0x04u /* 轨迹层求值失败 */
#define ARM_MPC_FAULT_TRACKING      0x08u /* 跟踪误差持续超限 (方向/零位配错的典型症状) */
#define ARM_MPC_FAULT_TORQUE        0x10u /* 力矩计算结果非有限值 */
#define ARM_MPC_FAULT_MOTOR_ERROR   0x20u /* 电机自身报错 (过压/欠压/过流/过温/过载等) */

/*
 * 单个关节的换算参数。全部是占位值, 上板前逐项标定。
 *
 * 达妙电机反馈的是减速器输出端的绝对位置 (零位用达妙上位机预先设置), 所以
 * 不需要减速比和上电归零:
 *   关节角 q = direction * 电机位置 + q_offset
 *   电机力矩 = direction * tau, 单位 N·m, 再限到 min(tau_max, 电机 T_MAX)
 *
 * 方向统一在这里处理, 电机实例的 motor_reverse_flag / feedback_reverse_flag
 * 保持 NORMAL, 两处都翻很容易翻错一次。
 */
typedef struct {
    float direction; /* +1 或 -1 */
    float q_offset;  /* 电机零位对应的关节角 (MDH 意义下), rad */
    float tau_max;   /* 总力矩 |tau_ff + dtau| 饱和上限, N·m */
} ArmMpcJointConfig;

typedef struct {
    InsArm arm;                                       /* 机械臂实例 (MDH + EC_Arm) */
    InsTraj traj;                                     /* 轨迹规划层 */
    DMMotorInstance *motors[ARM_MPC_JOINT_COUNT];     /* 关节电机; NULL 表示该关节未接 */
    ArmMpcJointConfig joint_cfg[ARM_MPC_JOINT_COUNT];

    float q_actual[ARM_MPC_JOINT_COUNT];              /* 关节角反馈, rad */
    float dq_actual[ARM_MPC_JOINT_COUNT];             /* 关节角速度反馈, rad/s */
    EC_JointStateReference ref;                       /* 本周期参考点 */

    float tau_ff[ARM_MPC_JOINT_COUNT];                /* RNEA 前馈力矩, Nm */
    float mpc_dtau[ARM_MPC_JOINT_COUNT];              /* MPC 修正力矩, Nm (未轮到的关节 hold) */
    float tau_total[ARM_MPC_JOINT_COUNT];             /* 饱和后的最终力矩, Nm */

    uint32_t tick;                                    /* 当前 tick (osKernelSysTick) */
    uint8_t mpc_joint_index;                          /* 错峰: 下一个要解的关节 */

    /* 目标输入, 由 ArmMpcTaskSetTarget() 写入, 一直保持到下一次写入 */
    float q_target[ARM_MPC_JOINT_COUNT];              /* 已夹进关节限位, rad; 零力矩下等于实际角 */
    uint8_t move;                                     /* ArmMpcMove */
    float move_time;                                  /* TIMED 的运动时长, s */

    /* 已经交给轨迹层的目标 */
    float q_goal[ARM_MPC_JOINT_COUNT];                /* rad */
    uint32_t goal_tick;                               /* 最近一次交给轨迹层的 tick */
    uint8_t goal_result;                              /* 轨迹层的返回值 EC_TrajResult, 0 为成功 */
    bool arrived;                                     /* 参考值已停在目标角 */

    uint8_t mode;                                     /* 上层请求的模式 ArmMpcMode */
    uint8_t fault_flags;                              /* 锁存的故障 ARM_MPC_FAULT_* */
    uint8_t motor_online;                             /* 按位: 电机在线且已使能 (未接的关节不置位) */
    bool limp;                                        /* 当前是否处于零力矩输出 */
    uint16_t track_err_ticks;                         /* 跟踪误差连续超限的周期数 */

    /* 诊断计数, 只增不减, 调试时加到 watch 里看。 */
    uint32_t mpc_fail_count;                          /* update/solve 调用失败 (沿用旧 dtau) */
    uint32_t mpc_unconverged_count;                   /* 未收敛但按仿真约定采用了迭代点 */
    uint32_t saturate_count;                          /* 总力矩被限幅 */
    uint32_t cycles_last;                             /* 本周期 RNEA+MPC 的 CPU 周期数 */
    uint32_t cycles_max;                              /* 历史最大值, 用来核对 F4 的实时预算 */

    bool initialized;
} ArmMpcTask;

/*
 * 初始化。可以在 RobotInit() 里 (关中断、还没有电机反馈时) 调用。
 * motors:    6 个关节电机, 未接的关节填 NULL。未接关节的"反馈"直接取参考值,
 *            这样可以一个关节一个关节地上电调试, 其余关节按理想跟踪参与 RNEA。
 * joint_cfg: 6 个关节的换算参数, 会拷贝一份; 任何一项非法都会初始化失败。
 * 初始化后处于 ZERO_FORCE, 需要上层切到 RUN 才会出力。
 */
bool ArmMpcTaskInit(ArmMpcTask *self,
                    DMMotorInstance *motors[ARM_MPC_JOINT_COUNT],
                    const ArmMpcJointConfig joint_cfg[ARM_MPC_JOINT_COUNT]);

/* 设置请求模式。切到 ZERO_FORCE 会清除锁存故障 (相当于复位按钮)。 */
void ArmMpcTaskSetMode(ArmMpcTask *self, ArmMpcMode mode);

/*
 * 设置目标角。不需要每周期都调, 不调就保持上一次的目标。
 * q_target:  6 个关节角, rad。超出关节限位的部分夹到限位上, 夹完的值写进
 *            self->q_target, 上层可以读回来; 含非有限值时整帧忽略。
 * move:      ArmMpcMove, 非法值按 FOLLOW 处理。
 * move_time: 只在 TIMED 下用, s, 限到轨迹层允许的段时长范围内。
 *            只改 move_time 不改目标角不会重新规划。
 * 零力矩期间设置的目标不起作用, 目标角每周期被改写成实际角。
 */
void ArmMpcTaskSetTarget(ArmMpcTask *self, ArmMpcMove move,
                         const float *q_target, float move_time);

/* 每个控制周期 (INS_ARM_CONTROL_TICK_MS) 调用一次。 */
void ArmMpcTaskRun(ArmMpcTask *self);

/* 轨迹层入口, 只读用 (取状态、计划); 设目标请走 ArmMpcTaskSetTarget()。 */
InsTraj *ArmMpcTaskTraj(ArmMpcTask *self);

#endif
