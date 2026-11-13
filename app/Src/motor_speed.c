#include "motor_speed.h"
#include "bsp_motor_encoder.h"
#include "chassis_config.h"
#include <stddef.h>

/* motor_feedback：本模块私有的双轮反馈，外部仅能获得副本。 */
static motor_feedback_t motor_feedback[MOTOR_COUNT];

bool motor_speed_init(void) {
    if (!bsp_encoder_init()) {
        return false;
    }
    motor_speed_reset();
    return true;
}

void motor_speed_update(uint32_t dt_ms) {
    if (dt_ms == 0U) {
        return;
    }
    float elapsed_ms = (float)dt_ms;
    float filter_alpha = elapsed_ms / (MOTOR_SPEED_FILTER_TIME_CONSTANT_MS + elapsed_ms);
    for (motor_id_t motor_id = MOTOR_LEFT; motor_id < MOTOR_COUNT; ++motor_id) {
        motor_feedback_t *feedback = &motor_feedback[motor_id];
        feedback->delta_counts = bsp_encoder_read_delta(motor_id);
        feedback->total_counts += feedback->delta_counts;
        feedback->raw_speed_mm_s =
            (float)feedback->delta_counts * MOTOR_DISTANCE_PER_COUNT_MM * 1000.0f / elapsed_ms;//计算原始轮速
        feedback->filtered_speed_mm_s +=
            filter_alpha * (feedback->raw_speed_mm_s - feedback->filtered_speed_mm_s);//更新滤波轮速
    }
}

bool motor_speed_get_feedback(motor_id_t motor_id, motor_feedback_t *feedback) {
    if ((unsigned int)motor_id >= MOTOR_COUNT || feedback == NULL) {
        return false;
    }
    *feedback = motor_feedback[motor_id];
    return true;
}//复制已经保存的反馈

float motor_speed_get_distance_mm(motor_id_t motor_id) {
    if ((unsigned int)motor_id >= MOTOR_COUNT) {
        return 0.0f;
    }
    return (float)motor_feedback[motor_id].total_counts * MOTOR_DISTANCE_PER_COUNT_MM;
}

void motor_speed_reset(void) {
    for (motor_id_t motor_id = MOTOR_LEFT; motor_id < MOTOR_COUNT; ++motor_id) {
        (void)bsp_encoder_read_delta(motor_id);
        motor_feedback[motor_id] = (motor_feedback_t){0};
    }
}
