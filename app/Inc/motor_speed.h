//
// Created by moonrise on 2026/11/12.
//

#ifndef CHYCHIC_PROJECT_MOTOR_SPEED_H
#define CHYCHIC_PROJECT_MOTOR_SPEED_H
#include <bsp_motor.h>
#include <stdint.h>

#define ENC_PPR         11 /* 光电编码器每转脉冲数 */
#define ENC_MULT        4  /* TI1 and TI2 = 4 倍频 */
#define GEAR_RATIO      35 /* 减速比 35:1 */
#define COUNTS_PER_REV  (ENC_PPR * ENC_MULT * GEAR_RATIO)   /* = 1540 ，轮子转一圈的计数*/

#define WHEEL_R       0.05f /* 轮子半径，单位：米 */
#define MM_PER_COUNT    (2.0f * 3.14159265f * WHEEL_R * 1000.0f / COUNTS_PER_REV) /* = 0.2040 mm/count ，单个计数的距离*/


typedef struct {
    int32_t d_counts;     /* 本周期计数增量 */
    int32_t total_counts; /* 累计计数 */
    float   velocity_raw;     /* 原始速度 mm/s */
    float   velocity;         /* 滤波后速度 mm/s */
} wheel_t;

extern wheel_t wheel[2];

void odom_update(uint32_t dt_ms);

#endif //CHYCHIC_PROJECT_MOTOR_SPEED_H
