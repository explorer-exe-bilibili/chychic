#ifndef PID_H
#define PID_H

#include <stdbool.h>

/** pid_gains_t：根据控制环的误差和输出单位整定的三项增益。 */
typedef struct {
    float kp; /* 比例增益。 */
    float ki; /* 积分增益，时间以秒计。 */
    float kd; /* 微分增益，时间以秒计。 */
} pid_gains_t;

/** pid_state_t：每个控制环单独持有的计算历史，使用前清零。 */
typedef struct {
    float integral;          /* 已乘积分增益的累计量，与输出同单位。 */
    float previous_error;    /* 上一次有效更新的误差。 */
    bool has_previous_error; /* 是否有可用于差分的历史误差。 */
} pid_state_t;

/** pid_reset：清除积分和历史误差。state：控制环状态。返回：无。 */
void pid_reset(pid_state_t *state);

/**
 * pid_step：按误差和实际周期计算带限幅的 PID 输出。
 * state/gains：非空状态和增益指针；error：本次有符号误差；dt_s：周期秒数；
 * feedforward：前馈量；output_min/output_max：总输出上下限；output：接收输出的非空指针。
 * 返回：成功为 true；无效输入或计算溢出时清除状态、输出零并返回 false。
 */
bool pid_step(pid_state_t *state, const pid_gains_t *gains, float error, float dt_s,
              float feedforward, float output_min, float output_max, float *output);

#endif
