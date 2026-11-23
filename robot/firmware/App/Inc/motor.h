/**
 * @file    motor.h
 * @brief   双轮驱动 + 编码器测速 + 速度闭环
 *
 * 约定：v > 0 车往前，w > 0 左转（和上位机一致）。
 * 差速运动学： v_l = v - w*b/2,  v_r = v + w*b/2
 */
#ifndef APP_MOTOR_H
#define APP_MOTOR_H

#include "main.h"

typedef struct
{
    TIM_HandleTypeDef *htim_pwm;
    uint32_t ch_l;
    uint32_t ch_r;

    GPIO_TypeDef *in1_port, *in3_port;   /* 左、右电机方向脚 A */
    uint16_t      in1_pin,  in3_pin;
    GPIO_TypeDef *in2_port, *in4_port;   /* 左、右电机方向脚 B */
    uint16_t      in2_pin,  in4_pin;

    TIM_HandleTypeDef *htim_enc_l;
    TIM_HandleTypeDef *htim_enc_r;

    /* 标定：轮子转 1 米编码器记多少数。实测方法见 firmware/README.md */
    float ticks_per_meter;

    /* 速度环 PI（1kHz 调用） */
    float kp;
    float ki;
    float pwm_max;          /* 一般等于 ARR */

    /* 状态 */
    float   target_l, target_r;   /* m/s */
    float   integ_l, integ_r;
    float   meas_l, meas_r;
    int32_t enc_l, enc_r;         /* 累计计数 */
    int32_t _prev_cnt_l, _prev_cnt_r;
    uint8_t enabled;
} motor_t;

void motor_init(motor_t *m);

/** 设置目标轮速（m/s），内部会限幅 */
void motor_set_target(motor_t *m, float v_l, float v_r);

/** 由 (v, w) 直接驱动，b 为轮距 m */
void motor_set_twist(motor_t *m, float v, float w, float wheel_base);

/** 1kHz 调用一次：更新测速、跑 PI、写 PWM */
void motor_update(motor_t *m, float dt);

/** 断使能并短路刹车 */
void motor_brake(motor_t *m);

/** 标定模式：不驱动电机，只累计编码器（用于 ticks_per_meter 实测） */
void motor_set_calib_mode(motor_t *m);

#endif /* APP_MOTOR_H */
