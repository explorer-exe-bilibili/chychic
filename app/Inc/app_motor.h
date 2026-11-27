#ifndef APP_MOTOR_H
#define APP_MOTOR_H

#include "motor_types.h"
#include "pid.h"
#include <stdbool.h>
#include <stdint.h>

/** app_motor_init：启动电机输出层。参数：无。返回：成功为 true。 */
bool app_motor_init(void);

/**
 * 读写单轮运行中的 PID 增益。motor_id 为轮编号，gains 为读写缓冲。
 * 写入仅接受有限非负的增益，修改后清除该轮的积分和误差历史。返回是否成功。
 */
bool app_motor_get_speed_pid(motor_id_t motor_id, pid_gains_t *gains);
bool app_motor_set_speed_pid(motor_id_t motor_id, const pid_gains_t *gains);

/**
 * app_motor_control_step：计算带死区补偿的 PWM，配置轮速 PID 时叠加闭环修正。
 * motor_id：轮编号；target_speed_mm_s：目标轮速；measured_speed_mm_s：实测轮速；
 * dt_s：实际控制周期，秒。返回：带符号 PWM 千分比，范围 [-1000, 1000]。
 */
int32_t app_motor_control_step(motor_id_t motor_id, float target_speed_mm_s,
                               float measured_speed_mm_s, float dt_s);

/**
 * app_motor_update：读取缓存轮速，分别计算并输出左右电机 PWM。
 * left_target_mm_s/right_target_mm_s：斜坡处理后的目标轮速；
 * dt_ms：本次实际周期，毫秒，0 时不更新。返回：无。
 */
void app_motor_update(float left_target_mm_s, float right_target_mm_s, uint32_t dt_ms);

/** app_motor_stop：清零电机输出并复位双轮 PID。参数：无。返回：无。 */
void app_motor_stop(void);

#endif
