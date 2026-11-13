//
// Created by moonrise on 2026/11/12.
//

#ifndef CHASSIS_H
#define CHASSIS_H
#include <stdint.h>

/* ====== 机械参数（实测） ====== */
#define TRACK_WIDTH_MM      150.0f   /* 左右轮中心距 mm */
#define MAX_WHEEL_MMPS      600.0f   /* 单轮满速 mm/s */

/* ====== PWM 参数 ====== */
#define PWM_ARR             999      /* 必须等于 TIM4->ARR */
#define DUTY_MAX            (PWM_ARR + 1)      /* = 1000，千分比满量程 */
#define DUTY_DEAD_L         180      /* 左轮死区，实测 */
#define DUTY_DEAD_R         180      /* 右轮死区，与左轮独立 */

/* ====== 运动参数 ====== */
#define ACCEL_MMPS2         6000.0f  /* 加速度上限 mm/s² */
#define RIGHT_MOTOR_INVERT  1        /* 右轮接反则设 1 */

void chassis_init(void);
void chassis_update(uint32_t dt_ms);          /* 每 10ms 调一次 */
void chassis_set_wheel(float v_l, float v_r); /* mm/s，带符号 */
void chassis_move(float v_mmps, float w_radps);
void chassis_brake(void);                     /* 软停：斜坡减速 */
void chassis_stop_now(void);                  /* 急停：立即断输出 */

/* 语义别名：只做薄封装，不写任何新逻辑 */
static inline void chassis_forward (float v) { chassis_move( v, 0.0f); }
static inline void chassis_backward(float v) { chassis_move(-v, 0.0f); }
static inline void chassis_spin    (float w) { chassis_move(0.0f, w); }
static inline void chassis_arc     (float v, float w) { chassis_move(v, w); }

#endif