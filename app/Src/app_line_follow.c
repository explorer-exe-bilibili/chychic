#include "app_line_follow.h"
#include "bsp_line_sensor.h"
#include "chassis.h"
#include "line_follow_config.h"
#include "pid.h"
#include <stddef.h>

_Static_assert(LINE_FOLLOW_ACTIVE_LOW == 0 || LINE_FOLLOW_ACTIVE_LOW == 1,
               "LINE_FOLLOW_ACTIVE_LOW must be 0 or 1");
_Static_assert(LINE_FOLLOW_REVERSE_SENSOR_ORDER == 0 || LINE_FOLLOW_REVERSE_SENSOR_ORDER == 1,
               "LINE_FOLLOW_REVERSE_SENSOR_ORDER must be 0 or 1");
_Static_assert(LINE_FOLLOW_STABLE_SAMPLES > 0U && LINE_FOLLOW_STABLE_SAMPLES <= UINT8_MAX,
               "LINE_FOLLOW_STABLE_SAMPLES must be in [1, 255]");
_Static_assert(LINE_FOLLOW_MAX_ACTIVE_SENSORS > 0U &&
                   LINE_FOLLOW_MAX_ACTIVE_SENSORS <= LINE_SENSOR_COUNT,
               "LINE_FOLLOW_MAX_ACTIVE_SENSORS must be in [1, 5]");

/* follow_status：模块私有状态；enabled 默认 false，上电不自动运动。 */
static line_follow_status_t follow_status;
/* candidate_line_mask / stable_samples：当前待确认线型及其连续出现次数。 */
static uint8_t candidate_line_mask;
static uint8_t stable_samples;
/* sensor_weights：左正右负，与 chassis 的逆时针正角速度约定一致。 */
static const float sensor_weights[LINE_SENSOR_COUNT] = {2.0f, 1.0f, 0.0f, -1.0f, -2.0f};

#ifdef LINE_FOLLOW_STEERING_PID_GAINS
/* steering_pid_gains / steering_pid_state：循迹转向增益和计算历史。 */
static const pid_gains_t steering_pid_gains = LINE_FOLLOW_STEERING_PID_GAINS;
static pid_state_t steering_pid_state;
/* steering_elapsed_s：距上一次 PID 更新的有效采样时间，包含等待线型确认的时间。 */
static float steering_elapsed_s;
#endif

/**
 * line_follow_normalize：将 OUT 原始电平转换为从左到右的黑线位图。
 * raw_levels：OUT1～OUT5 原始位图。返回：bit0 最左、bit4 最右，1 表示黑线。
 */
static uint8_t line_follow_normalize(uint8_t raw_levels) {
    uint8_t active_mask = raw_levels & LINE_SENSOR_MASK;
    if (LINE_FOLLOW_ACTIVE_LOW) {
        active_mask = (uint8_t)(~active_mask) & LINE_SENSOR_MASK;
    }
    if (!LINE_FOLLOW_REVERSE_SENSOR_ORDER) {
        return active_mask;
    }
    uint8_t reversed_mask = 0U; /* 反转后按车体左到右排列的黑线位图。 */
    for (uint8_t sensor_index = 0U; sensor_index < LINE_SENSOR_COUNT; ++sensor_index) {
        if ((active_mask & (1U << sensor_index)) != 0U) {
            reversed_mask |= (uint8_t)(1U << (LINE_SENSOR_COUNT - 1U - sensor_index));
        }
    }
    return reversed_mask;
}

/**
 * line_follow_clear_command：清零输出目标和候选采样，设置指定状态。
 * state：停车后记录的状态。返回：无。不改变 enabled，以便丢线恢复后重新捕获。
 */
static void line_follow_clear_command(line_follow_state_t state) {
    candidate_line_mask = 0U;
    stable_samples = 0U;
    follow_status.state = state;
    follow_status.position_error = 0.0f;
    follow_status.linear_speed_mm_s = 0.0f;
    follow_status.angular_speed_rad_s = 0.0f;
#ifdef LINE_FOLLOW_STEERING_PID_GAINS
    pid_reset(&steering_pid_state);
    steering_elapsed_s = 0.0f;
#endif
    chassis_stop();
}

void app_line_follow_init(void) {
    follow_status = (line_follow_status_t){0};
    line_follow_clear_command(LINE_FOLLOW_DISABLED);
}

void app_line_follow_start(void) {
    follow_status.enabled = true;
    line_follow_clear_command(LINE_FOLLOW_ACQUIRING);
}

void app_line_follow_stop(void) {
    follow_status.enabled = false;
    line_follow_clear_command(LINE_FOLLOW_DISABLED);
}

void app_line_follow_update(uint32_t dt_ms)
{
    if (dt_ms == 0U) {
        return;
    }
    follow_status.raw_levels = bsp_line_sensor_read_raw();
    follow_status.line_mask = line_follow_normalize(follow_status.raw_levels);
    follow_status.active_sensor_count = 0U;

    float weighted_position_sum = 0.0f;  /* 检测到黑线的探头权重之和。 */
    uint8_t line_segment_count = 0U;     /* 相邻黑线探头算一段，分开的黑线算多段。 */
    bool previous_sensor_active = false; /* 前一只探头是否识别到黑线。 */
    for (uint8_t sensor_index = 0U; sensor_index < LINE_SENSOR_COUNT; ++sensor_index) {
        bool sensor_active = (follow_status.line_mask & (1U << sensor_index)) != 0U;
        if (sensor_active) {
            ++follow_status.active_sensor_count;
            weighted_position_sum += sensor_weights[sensor_index];
            if (!previous_sensor_active) {
                ++line_segment_count;
            }
        }
        previous_sensor_active = sensor_active;
    }

    /* 禁用时仍采样供调试，但不抢占手动 chassis_set_velocity 设置的目标。 */
    if (!follow_status.enabled) {
        follow_status.state = LINE_FOLLOW_DISABLED;
        return;
    }
    if (follow_status.active_sensor_count == 0U) {
        line_follow_clear_command(LINE_FOLLOW_LOST);
        return;
    }
    if (line_segment_count > 1U) {
        line_follow_clear_command(LINE_FOLLOW_AMBIGUOUS);
        return;
    }
    if (follow_status.active_sensor_count > LINE_FOLLOW_MAX_ACTIVE_SENSORS) {
        line_follow_clear_command(LINE_FOLLOW_WIDE_LINE);
        return;
    }
    follow_status.position_error = weighted_position_sum / (float)follow_status.active_sensor_count;

#ifdef LINE_FOLLOW_STEERING_PID_GAINS
    float dt_s = (float)dt_ms / 1000.0f;
    steering_elapsed_s += dt_s;
#endif

    if (follow_status.line_mask != candidate_line_mask) {
        candidate_line_mask = follow_status.line_mask;
        stable_samples = 1U;
    } else if (stable_samples < LINE_FOLLOW_STABLE_SAMPLES) {
        ++stable_samples;
    }
    if (stable_samples < LINE_FOLLOW_STABLE_SAMPLES) {
        /* 初次捕获时停住；已经循迹时保持旧目标，过滤短暂的有效线型跳变。 */
        if (follow_status.state != LINE_FOLLOW_TRACKING) {
            follow_status.state = LINE_FOLLOW_ACQUIRING;
        }
        return;
    }

    float angular_speed_rad_s;
#ifdef LINE_FOLLOW_STEERING_PID_GAINS
    /* 首次捕获从当前采样周期起积分；循迹中计入等待线型确认的周期。 */
    float control_dt_s = follow_status.state == LINE_FOLLOW_TRACKING ? steering_elapsed_s : dt_s;
    steering_elapsed_s = 0.0f;
    if (!pid_step(&steering_pid_state, &steering_pid_gains, follow_status.position_error,
                  control_dt_s, 0.0f, -LINE_FOLLOW_MAX_ANGULAR_SPEED_RAD_S,
                  LINE_FOLLOW_MAX_ANGULAR_SPEED_RAD_S, &angular_speed_rad_s)) {
        line_follow_clear_command(LINE_FOLLOW_ACQUIRING);
        return;
    }
#else
    angular_speed_rad_s = follow_status.position_error * LINE_FOLLOW_STEERING_KP_RAD_S;
    if (angular_speed_rad_s > LINE_FOLLOW_MAX_ANGULAR_SPEED_RAD_S) {
        angular_speed_rad_s = LINE_FOLLOW_MAX_ANGULAR_SPEED_RAD_S;
    } else if (angular_speed_rad_s < -LINE_FOLLOW_MAX_ANGULAR_SPEED_RAD_S) {
        angular_speed_rad_s = -LINE_FOLLOW_MAX_ANGULAR_SPEED_RAD_S;
    }
#endif
    follow_status.state = LINE_FOLLOW_TRACKING;
    follow_status.linear_speed_mm_s = LINE_FOLLOW_FORWARD_SPEED_MM_S;
    follow_status.angular_speed_rad_s = angular_speed_rad_s;
    /* 只下发目标；测速、斜坡和 PWM 输出仍由主循环下一步的 chassis_update 完成。 */
    chassis_set_velocity(follow_status.linear_speed_mm_s, follow_status.angular_speed_rad_s);
}

bool app_line_follow_get_status(line_follow_status_t *status) {
    if (status == NULL) {
        return false;
    }
    *status = follow_status;
    return true;/* 复制状态副本，避免外部直接修改。 */
}
