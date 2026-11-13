//
// Created by moonrise on 2026/11/12.
//

#include "../Inc/chassis.h"
#include "chassis.h"
#include "bsp_motor.h"
#include <math.h>

static float s_tgt_l = 0.0f, s_tgt_r = 0.0f;   /* 目标速度 mm/s */
static float s_out_l = 0.0f, s_out_r = 0.0f;   /* 斜坡后指令 mm/s */

void chassis_init(void)
{
    s_tgt_l = s_tgt_r = 0.0f;
    s_out_l = s_out_r = 0.0f;
    left_motor_set_speed(0);
    right_motor_set_speed(0);
}

/* mm/s → 占空比（开环 + 死区补偿）；上 PID 时只替换这一个函数 */
static int32_t mmps_to_duty(float v_mmps, int32_t dead)
{
    if (v_mmps >  MAX_WHEEL_MMPS) v_mmps =  MAX_WHEEL_MMPS;   /* ① 饱和 */
    if (v_mmps < -MAX_WHEEL_MMPS) v_mmps = -MAX_WHEEL_MMPS;

    float mag = (v_mmps >= 0.0f) ? v_mmps : -v_mmps;          /* ② 幅值 */
    if (mag < 1.0f) return 0;                                 /* ③ 死区下沿 */

    int32_t duty = dead +                                     /* ④ 线性映射 */
        (int32_t)((DUTY_MAX - dead) * mag / MAX_WHEEL_MMPS + 0.5f);
    return (v_mmps >= 0.0f) ? duty : -duty;                   /* ⑤ 还原符号 */
}

void chassis_set_wheel(float v_l, float v_r)
{
    /* 等比缩放：保持 v:ω 比例，避免限幅扭曲转弯半径 */
    float m = fmaxf(fabsf(v_l), fabsf(v_r));
    if (m > MAX_WHEEL_MMPS) {
        float k = MAX_WHEEL_MMPS / m;
        v_l *= k;
        v_r *= k;
    }
    s_tgt_l = v_l;
    s_tgt_r = v_r;
}

void chassis_move(float v_mmps, float w_radps)
{
    float half = w_radps * TRACK_WIDTH_MM * 0.5f;   /* 单位 mm/s */
    chassis_set_wheel(v_mmps - half, v_mmps + half);
}

void chassis_brake(void)      { s_tgt_l = s_tgt_r = 0.0f; }

void chassis_stop_now(void)
{
    s_tgt_l = s_tgt_r = 0.0f;
    s_out_l = s_out_r = 0.0f;
    left_motor_set_speed(0);
    right_motor_set_speed(0);
}

void chassis_update(uint32_t dt_ms)
{
    if (dt_ms == 0) return;

    float max_step = ACCEL_MMPS2 * (float)dt_ms / 1000.0f;   /* 每拍允许的变化量 */

    float dl = s_tgt_l - s_out_l;
    if (dl >  max_step) dl =  max_step;
    if (dl < -max_step) dl = -max_step;
    s_out_l += dl;

    float dr = s_tgt_r - s_out_r;
    if (dr >  max_step) dr =  max_step;
    if (dr < -max_step) dr = -max_step;
    s_out_r += dr;

    int32_t duty_l =  mmps_to_duty(s_out_l, DUTY_DEAD_L);
    int32_t duty_r =  mmps_to_duty(s_out_r, DUTY_DEAD_R);
#if RIGHT_MOTOR_INVERT
    duty_r = -duty_r;
#endif
    left_motor_set_speed(duty_l);
    right_motor_set_speed(duty_r);
}