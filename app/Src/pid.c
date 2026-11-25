#include "pid.h"
#include <math.h>

void pid_reset(pid_state_t *state) {
    *state = (pid_state_t){0};
}

bool pid_step(pid_state_t *state, const pid_gains_t *gains, float error, float dt_s,
              float feedforward, float output_min, float output_max, float *output) {
    if (!isfinite(error) || !isfinite(dt_s) || dt_s <= 0.0f || !isfinite(feedforward) ||
        !isfinite(output_min) || !isfinite(output_max) || output_min > output_max ||
        !isfinite(gains->kp) || !isfinite(gains->ki) || !isfinite(gains->kd)) {
        pid_reset(state);
        *output = 0.0f;
        return false;
    }

    float derivative = 0.0f; /* 复位后的首个误差没有历史差分。 */
    if (state->has_previous_error && gains->kd != 0.0f) {
        derivative = gains->kd * (error - state->previous_error) / dt_s;
    }
    float proportional_and_derivative = feedforward + gains->kp * error + derivative;
    float previous_output = proportional_and_derivative + state->integral;
    float integral_step = gains->ki * error * dt_s;
    if (!isfinite(previous_output) || !isfinite(integral_step)) {
        pid_reset(state);
        *output = 0.0f;
        return false;
    }

    /* 只在积分变化方向还有输出余量时累加，已饱和时仍可向限幅内回退。 */
    if (integral_step > 0.0f && previous_output < output_max) {
        state->integral += fminf(integral_step, output_max - previous_output);
    } else if (integral_step < 0.0f && previous_output > output_min) {
        state->integral += fmaxf(integral_step, output_min - previous_output);
    }
    float integral_limit = fmaxf(fabsf(output_min), fabsf(output_max));
    state->integral = fmaxf(-integral_limit, fminf(integral_limit, state->integral));
    state->previous_error = error;
    state->has_previous_error = true;
    *output = fmaxf(output_min, fminf(output_max, proportional_and_derivative + state->integral));
    return true;
}
