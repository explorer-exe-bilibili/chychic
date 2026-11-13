#ifndef MOTOR_SPEED_H
#define MOTOR_SPEED_H

#include "motor_types.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int32_t delta_counts;      /* 本次采样计数增量。 */
    int64_t total_counts;      /* 自初始化或复位后的累计计数。 */
    float raw_speed_mm_s;      /* 未滤波轮速，毫米/秒。 */
    float filtered_speed_mm_s; /* 低通滤波轮速，毫米/秒，预留给速度 PID。 */
} motor_feedback_t;

/** motor_speed_init：启动编码器并清零反馈。成功为 true。 */
bool motor_speed_init(void);

/**
 * motor_speed_update：统一采样左右轮并更新速度与累计计数。
 * dt_ms：距上次采样的实际毫秒数，0 时不采样。
 * 应仅由 chassis_update 调用一次，保持时间间隔与计数区间一致。
 */
void motor_speed_update(uint32_t dt_ms);

/**
 * motor_speed_get_feedback：获取缓存副本，不读取或消费硬件计数。
 * motor_id：轮编号；feedback：接收反馈的非空指针。
 * 返回：成功为 true，无效编号或空指针为 false。
 */
bool motor_speed_get_feedback(motor_id_t motor_id, motor_feedback_t *feedback);

/**
 * motor_speed_get_distance_mm：从唯一累计计数计算行驶距离。
 * motor_id：轮编号。返回：带符号毫米数，无效编号为 0。
 */
float motor_speed_get_distance_mm(motor_id_t motor_id);

/**
 * motor_speed_reset：丢弃未采样增量，并清零双轮累计计数和滤波状态。
 * 停车后调用，并重新建立主循环采样时间基准。
 */
void motor_speed_reset(void);
/**
*初始化时：
motor_speed_init()
    → 启动编码器
    → 建立基准、清零数据

*每个控制周期：
motor_speed_update(dt_ms)
    → 读取增量
    → 累计计数
    → 计算原始速度
    → 更新滤波速度

*需要使用反馈时：
motor_speed_get_feedback(...)
    → 取得保存好的反馈副本

*需要查询位置变化时：
motor_speed_get_distance_mm(...)
    → 将保存的累计计数换算为毫米 */
#endif
