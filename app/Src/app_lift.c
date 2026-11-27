#include "app_lift.h"
#include "bsp_lift.h"
#include "lift_config.h"

#include <math.h>

static float counts_per_mm;        /* 标定后的每毫米计数。 */
static uint16_t max_height_mm;     /* 从手动零点向上允许的最大行程。 */
static int32_t max_height_counts;  /* 软件行程转换为计数后的上限。 */
static uint32_t zero_count;        /* 手动设零时的原始计数。 */
static bool referenced;            /* 是否具有可信零点。 */
static uint16_t target_height_mm;  /* 最近接受的目标高度。 */
static int32_t target_counts;      /* 相对于零点的目标计数。 */
static int32_t best_counts;        /* 本次运动向目标推进最远时的位置。 */
static int32_t move_direction;     /* +1 上升，-1 下降，仅在运动中使用。 */
static uint32_t move_start_ms;     /* 本次运动开始的时间。 */
static uint32_t last_progress_ms;  /* 最近一次朝目标推进的时间。 */
static lift_state_t lift_state;     /* 当前定位状态。 */
static lift_fault_t lift_fault;     /* 最近一次定位故障。 */

/** 将回绕计数转换为有符号值；count 为无符号计数差。 */
static int32_t signed_count(uint32_t count) {
    if (count <= INT32_MAX) {
        return (int32_t)count;
    }
    return -1 - (int32_t)(UINT32_MAX - count);
}

/** 返回相对于设零位置的编码器计数。 */
static int32_t position_counts(void) {
    return signed_count(bsp_lift_encoder_count() - zero_count);
}

/** count 为相对零点的计数，返回是否处于已标定的软件行程中。 */
static bool within_travel(int32_t count) {
    return count >= 0 && count <= max_height_counts;
}

/** 返回到位容差，至少保留计数取整对应的半个计数。 */
static float tolerance_counts(void) {
    return fmaxf(0.5f, LIFT_POSITION_TOLERANCE_MM * counts_per_mm);
}

bool app_lift_init(void) {
    return bsp_lift_init();
}

lift_command_result_t app_lift_configure(float new_counts_per_mm, uint16_t new_max_height_mm) {
    if (lift_state == LIFT_MOVING) {
        return LIFT_COMMAND_BUSY;
    }

    float travel_counts = new_counts_per_mm * new_max_height_mm;
    if (!isfinite(travel_counts) || travel_counts < 1.0f || travel_counts >= 2147483648.0f) {
        return LIFT_COMMAND_INVALID;
    }

    counts_per_mm = new_counts_per_mm;
    max_height_mm = new_max_height_mm;
    max_height_counts = (int32_t)lroundf(travel_counts);
    referenced = false;
    target_height_mm = 0;
    lift_fault = LIFT_FAULT_NONE;
    lift_state = LIFT_UNREFERENCED;
    return LIFT_COMMAND_OK;
}

lift_command_result_t app_lift_set_zero(void) {
    if (lift_state == LIFT_MOVING) {
        return LIFT_COMMAND_BUSY;
    }
    if (max_height_mm == 0U) {
        return LIFT_COMMAND_UNCONFIGURED;
    }

    zero_count = bsp_lift_encoder_count();
    referenced = true;
    target_height_mm = 0;
    lift_fault = LIFT_FAULT_NONE;
    lift_state = LIFT_IDLE;
    return LIFT_COMMAND_OK;
}

lift_command_result_t app_lift_move(uint16_t target_mm, uint32_t now_ms) {
    if (lift_state == LIFT_MOVING) {
        return LIFT_COMMAND_BUSY;
    }
    if (max_height_mm == 0U) {
        return LIFT_COMMAND_UNCONFIGURED;
    }
    if (!referenced) {
        return LIFT_COMMAND_NEEDS_ZERO;
    }

    int32_t current_counts = position_counts();
    if (!within_travel(current_counts)) {
        app_lift_stop(LIFT_FAULT_RANGE);
        return LIFT_COMMAND_NEEDS_ZERO;
    }
    if (target_mm > max_height_mm) {
        return LIFT_COMMAND_INVALID;
    }

    target_height_mm = target_mm;
    target_counts = (int32_t)lroundf((float)target_mm * counts_per_mm);
    lift_fault = LIFT_FAULT_NONE;
    int32_t error_counts = target_counts - current_counts;
    if (fabsf((float)error_counts) <= tolerance_counts()) {
        lift_state = LIFT_AT_TARGET;
        return LIFT_COMMAND_OK;
    }

    move_direction = error_counts > 0 ? 1 : -1;
    best_counts = current_counts;
    move_start_ms = now_ms;
    last_progress_ms = now_ms;
    lift_state = LIFT_MOVING;
    return LIFT_COMMAND_OK;
}

void app_lift_stop(lift_fault_t reason) {
    if (lift_state != LIFT_MOVING && reason != LIFT_FAULT_RANGE) {
        return;
    }
    bsp_lift_set_pwm(0);
    lift_fault = reason;
    lift_state = reason == LIFT_FAULT_NONE ? LIFT_IDLE : LIFT_FAULT;
    if (reason == LIFT_FAULT_DIRECTION || reason == LIFT_FAULT_NO_PROGRESS ||
        reason == LIFT_FAULT_RANGE) {
        referenced = false;
    }
}

void app_lift_update(uint32_t now_ms) {
    if (lift_state != LIFT_MOVING) {
        return;
    }

    int32_t current_counts = position_counts();
    if (!within_travel(current_counts)) {
        app_lift_stop(LIFT_FAULT_RANGE);
        return;
    }

    int32_t remaining_counts = move_direction * (target_counts - current_counts);
    float tolerance = tolerance_counts();
    if ((float)remaining_counts < -tolerance) {
        app_lift_stop(LIFT_FAULT_OVERSHOOT);
        return;
    }
    if ((float)remaining_counts <= tolerance) {
        bsp_lift_set_pwm(0);
        lift_state = LIFT_AT_TARGET;
        return;
    }
    if ((uint32_t)(now_ms - move_start_ms) >= LIFT_MOTION_TIMEOUT_MS) {
        app_lift_stop(LIFT_FAULT_TIMEOUT);
        return;
    }

    int32_t progress_counts = move_direction * (current_counts - best_counts);
    if (progress_counts < -1) {
        app_lift_stop(LIFT_FAULT_DIRECTION);
        return;
    }
    if (progress_counts > 0) {
        best_counts = current_counts;
        last_progress_ms = now_ms;
    } else if ((uint32_t)(now_ms - last_progress_ms) >= LIFT_PROGRESS_TIMEOUT_MS) {
        app_lift_stop(LIFT_FAULT_NO_PROGRESS);
        return;
    }

    int16_t duty_permille = (float)remaining_counts <= LIFT_APPROACH_DISTANCE_MM * counts_per_mm
                                 ? LIFT_APPROACH_PWM_PERMILLE
                                 : LIFT_RUN_PWM_PERMILLE;
    bsp_lift_set_pwm((int16_t)(move_direction * duty_permille));
}

lift_status_t app_lift_get_status(void) {
    uint32_t count = bsp_lift_encoder_count();
    return (lift_status_t){
        .position_mm = referenced ? (float)signed_count(count - zero_count) / counts_per_mm : 0.0f,
        .target_mm = target_height_mm,
        .encoder_count = signed_count(count),
        .state = lift_state,
        .fault = lift_fault,
        .configured = max_height_mm != 0U,
        .referenced = referenced,
    };
}
