#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdbool.h>
#include <stdint.h>

/**
 * chassis_init：启动电机与编码器，清零速度目标和反馈。
 * 返回：成功为 true；失败保持零输出并返回 false。
 * 所有接口由同一个主循环上下文调用。
 */
bool chassis_init(void);

/**
 * chassis_set_wheel_speeds：设置左右轮目标线速度，等比限幅后保存。
 * left_speed_mm_s/right_speed_mm_s：单位毫米/秒，向前滚动为正。
 */
void chassis_set_wheel_speeds(float left_speed_mm_s, float right_speed_mm_s);

/**
 * chassis_set_velocity：将车体线速度和角速度解算为左右轮目标。
 * linear_speed_mm_s：前进为正，毫米/秒；angular_speed_rad_s：俯视逆时针为正，单位弧度/秒。
 */
void chassis_set_velocity(float linear_speed_mm_s, float angular_speed_rad_s);

/**
 * chassis_update：先采样反馈，再执行指令斜坡和电机控制。
 * dt_ms：距上次更新的实际毫秒数；0 或未成功初始化时不执行。返回：无。
 */
void chassis_update(uint32_t dt_ms);

/**
 * chassis_request_stop：将目标设为零，通过后续 update 按斜坡减速。
 * 参数：无。返回：无。此接口不表示电机驱动芯片的主动刹车模式。
 */
void chassis_request_stop(void);

/**
 * chassis_stop：清零目标、斜坡状态和 PWM 指令，不等待斜坡。
 * 参数：无。返回：无。PWM 在下个更新事件生效，机械停车仍受惯性影响。
 */
void chassis_stop(void);

#endif
