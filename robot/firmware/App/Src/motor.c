/**
 * @file    motor.c
 * @note    依赖 CubeMX 生成的 main.h（含 TIM_HandleTypeDef / GPIO 定义）。
 *          如果不用 HAL，只需改 _pwm_write / _dir_write 两个函数。
 */
#include "motor.h"

#define MOTOR_V_MAX   0.80f    /* 轮速上限 m/s，超了会打滑 */

/* ---- 底层输出（换平台只改这两个） -------------------------------------- */

static void _pwm_write(motor_t *m, float l, float r)
{
    if (l < 0.0f) l = 0.0f;
    if (r < 0.0f) r = 0.0f;
    __HAL_TIM_SET_COMPARE(m->htim_pwm, m->ch_l, (uint32_t)l);
    __HAL_TIM_SET_COMPARE(m->htim_pwm, m->ch_r, (uint32_t)r);
}

static void _dir_write(motor_t *m, int8_t dl, int8_t dr)
{
    /* L298N：IN1/IN2 一高一低定方向，同为低 = 滑行 */
    HAL_GPIO_WritePin(m->in1_port, m->in1_pin, (dl > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(m->in2_port, m->in2_pin, (dl < 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(m->in3_port, m->in3_pin, (dr > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(m->in4_port, m->in4_pin, (dr < 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static float _clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float _absf(float x) { return (x < 0.0f) ? -x : x; }

/* ---- 初始化 ------------------------------------------------------------ */

void motor_init(motor_t *m)
{
    if (m->ticks_per_meter <= 0.0f)
        m->ticks_per_meter = 6000.0f;   /* 保险值，必须实测替换 */
    if (m->pwm_max <= 0.0f)
        m->pwm_max = 1000.0f;
    if (m->kp <= 0.0f) m->kp = 120.0f;
    if (m->ki <= 0.0f) m->ki = 900.0f;

    m->target_l = m->target_r = 0.0f;
    m->integ_l = m->integ_r = 0.0f;
    m->meas_l = m->meas_r = 0.0f;
    m->enc_l = m->enc_r = 0;
    m->_prev_cnt_l = (int32_t)__HAL_TIM_GET_COUNTER(m->htim_enc_l);
    m->_prev_cnt_r = (int32_t)__HAL_TIM_GET_COUNTER(m->htim_enc_r);
    m->enabled = 0;

    HAL_TIM_PWM_Start(m->htim_pwm, m->ch_l);
    HAL_TIM_PWM_Start(m->htim_pwm, m->ch_r);
    HAL_TIM_Encoder_Start(m->htim_enc_l, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(m->htim_enc_r, TIM_CHANNEL_ALL);

    _pwm_write(m, 0.0f, 0.0f);
    _dir_write(m, 0, 0);
}

/* ---- 目标 -------------------------------------------------------------- */

void motor_set_target(motor_t *m, float v_l, float v_r)
{
    m->target_l = _clampf(v_l, -MOTOR_V_MAX, MOTOR_V_MAX);
    m->target_r = _clampf(v_r, -MOTOR_V_MAX, MOTOR_V_MAX);
    m->enabled = 1;
}

void motor_set_twist(motor_t *m, float v, float w, float wheel_base)
{
    /* 兼容麦轮：若以后要走平移，再扩展 vy；现在先按差速处理 */
    float half = 0.5f * w * wheel_base;
    motor_set_target(m, v - half, v + half);
}

/* ---- 控制 -------------------------------------------------------------- */

void motor_update(motor_t *m, float dt)
{
    if (dt <= 0.0f)
        return;

    /* 1) 读编码器（计数器 16 位会回绕，转 int16 差分自动处理） */
    int32_t cnt_l = (int32_t)__HAL_TIM_GET_COUNTER(m->htim_enc_l);
    int32_t cnt_r = (int32_t)__HAL_TIM_GET_COUNTER(m->htim_enc_r);
    int32_t d_l = (int16_t)(cnt_l - m->_prev_cnt_l);
    int32_t d_r = (int16_t)(cnt_r - m->_prev_cnt_r);
    m->_prev_cnt_l = cnt_l;
    m->_prev_cnt_r = cnt_r;
    m->enc_l += d_l;
    m->enc_r += d_r;

    m->meas_l = (float)d_l / m->ticks_per_meter / dt;
    m->meas_r = (float)d_r / m->ticks_per_meter / dt;

    if (!m->enabled)
    {
        _pwm_write(m, 0.0f, 0.0f);
        return;
    }

    /* 2) 速度 PI */
    float err_l = m->target_l - m->meas_l;
    float err_r = m->target_r - m->meas_r;

    m->integ_l += err_l * dt;
    m->integ_r += err_r * dt;
    /* 抗饱和：积分项本身限幅到 30% 占空比 */
    m->integ_l = _clampf(m->integ_l, -0.3f, 0.3f);
    m->integ_r = _clampf(m->integ_r, -0.3f, 0.3f);

    float out_l = m->kp * err_l + m->ki * m->integ_l;
    float out_r = m->kp * err_r + m->ki * m->integ_r;

    /* 3) 方向和占空比 */
    int8_t dir_l = (out_l >= 0.0f) ? 1 : -1;
    int8_t dir_r = (out_r >= 0.0f) ? 1 : -1;

    /* 目标为 0 时别抖：给小占空比补一点，克服静摩擦 */
    float pwm_l = _absf(out_l);
    float pwm_r = _absf(out_r);
    if (_absf(m->target_l) > 0.01f && pwm_l < 60.0f) pwm_l = 60.0f;
    if (_absf(m->target_r) > 0.01f && pwm_r < 60.0f) pwm_r = 60.0f;
    pwm_l = _clampf(pwm_l, 0.0f, m->pwm_max);
    pwm_r = _clampf(pwm_r, 0.0f, m->pwm_max);

    _dir_write(m, dir_l, dir_r);
    _pwm_write(m, pwm_l, pwm_r);
}

void motor_brake(motor_t *m)
{
    m->enabled = 0;
    m->target_l = m->target_r = 0.0f;
    m->integ_l = m->integ_r = 0.0f;
    _pwm_write(m, 0.0f, 0.0f);
    _dir_write(m, 0, 0);
}

void motor_set_calib_mode(motor_t *m)
{
    m->enabled = 0;
    _pwm_write(m, 0.0f, 0.0f);
    _dir_write(m, 0, 0);
}
