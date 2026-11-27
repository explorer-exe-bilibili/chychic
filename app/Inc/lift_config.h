#ifndef LIFT_CONFIG_H
#define LIFT_CONFIG_H

/* 实测极性：1 表示 CH1 驱动上升。编码器 B 相在上升时若是高电平，将对应宏改成 1。 */
#define LIFT_UP_ON_CH1 1
#define LIFT_ENCODER_UP_B_HIGH 0

/* 初次低速联调的输出和停机参数，按驱动、电机和负载实测调整。 */
#define LIFT_RUN_PWM_PERMILLE 300
#define LIFT_APPROACH_PWM_PERMILLE 180
#define LIFT_APPROACH_DISTANCE_MM 5.0f
#define LIFT_POSITION_TOLERANCE_MM 0.5f
#define LIFT_MOTION_TIMEOUT_MS 10000U
#define LIFT_PROGRESS_TIMEOUT_MS 500U

#endif
