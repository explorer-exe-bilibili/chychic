#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

/* 实际轮组应按实物重新核验。 */
#define CHASSIS_TRACK_WIDTH_MM 234.0f          /* 两驱动轮中心距，毫米。 */
#define CHASSIS_MAX_WHEEL_SPEED_MM_S 600.0f    /* 单轮目标速度上限，毫米/秒。 */
#define CHASSIS_MAX_ACCELERATION_MM_S2 6000.0f /* 单轮指令加速度上限，毫米/秒²。 */
#define CHASSIS_CONTROL_PERIOD_MS 10U          /* 主循环目标更新间隔，毫秒。 */
#define MOTOR_LEFT_DEADZONE_PERMILLE 270       /* 左轮开环启动补偿，千分比。 */
#define MOTOR_RIGHT_DEADZONE_PERMILLE 270      /* 右轮开环启动补偿，千分比。 */
#define MOTOR_ZERO_SPEED_THRESHOLD_MM_S 1.0f   /* 低于此目标速度时输出零。 */

/*
 * 轮速整定时，可分别定义 MOTOR_LEFT_SPEED_PID_GAINS 和 MOTOR_RIGHT_SPEED_PID_GAINS，
 * 使用带 .kp、.ki、.kd 的 pid_gains_t 初始化列表。误差单位为毫米/秒，输出为 PWM 千分比。
 * 对应宏未定义时，沿用开环映射。
 */

#ifndef MOTOR_LEFT_SPEED_PID_GAINS
#define MOTOR_LEFT_SPEED_PID_GAINS { .kp = 0.0f, .ki = 0.0f, .kd = 0.0f }
#endif
#ifndef MOTOR_RIGHT_SPEED_PID_GAINS
#define MOTOR_RIGHT_SPEED_PID_GAINS { .kp = 0.0f, .ki = 0.0f, .kd = 0.0f }
#endif

#define MOTOR_ENCODER_PULSES_PER_REV 11U   /* 电机轴单通道每转脉冲数，待核验。 */
#define MOTOR_ENCODER_QUADRATURE_FACTOR 4U /* TI12 编码器模式的四倍频。 */
#define MOTOR_GEAR_RATIO 35U               /* 电机轴转速 / 轮轴转速。 */
#define MOTOR_COUNTS_PER_WHEEL_REV                                                                 \
    (MOTOR_ENCODER_PULSES_PER_REV * MOTOR_ENCODER_QUADRATURE_FACTOR * MOTOR_GEAR_RATIO)
#define MOTOR_WHEEL_RADIUS_MM 34.0f /* 轮子半径，毫米，不是直径。 */
#define MOTOR_DISTANCE_PER_COUNT_MM                                                                \
    (2.0f * 3.14159265f * MOTOR_WHEEL_RADIUS_MM / MOTOR_COUNTS_PER_WHEEL_REV)
/* 采用 alpha=dt/(tau+dt)，在 dt=10ms 时保留原工程 alpha=0.3 的滤波效果。 */
#define MOTOR_SPEED_FILTER_TIME_CONSTANT_MS (70.0f / 3.0f)

#endif
