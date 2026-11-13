//
// Created by moonrise on 2026/11/12.
//
#include "motor_speed.h"
#include "bsp_motor_encoder.h"

wheel_t wheel[2];

void odom_update(uint32_t dt_ms)
{
    if (dt_ms == 0) return;                     /* 防除零 */

    for (uint8_t i = 0; i < 2; i++) {
        wheel[i].d_counts      = encoder_get_delta(i);
        wheel[i].total_counts += wheel[i].d_counts;

        /* v = 位移 / 时间，单位 mm/s */
        wheel[i].velocity_raw = wheel[i].d_counts * MM_PER_COUNT * 1000.0f / (float)dt_ms;

        /* 一阶低通：alpha 越大越灵敏，0.3 对应约 30ms 时间常数 */
        wheel[i].velocity += (wheel[i].velocity_raw - wheel[i].velocity) * 0.3f;
    }
}

float wheel_total_mm(uint8_t i) { return wheel[i].total_counts * MM_PER_COUNT; }