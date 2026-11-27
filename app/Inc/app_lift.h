#ifndef APP_LIFT_H
#define APP_LIFT_H

#include <stdbool.h>
#include <stdint.h>

/* 枚举数值用于 UART1 协议。 */
typedef enum {
    LIFT_UNREFERENCED = 0, /* 尚未建立零点。 */
    LIFT_IDLE = 1,         /* 已设零，当前静止。 */
    LIFT_MOVING = 2,       /* 正在执行一条定位指令。 */
    LIFT_AT_TARGET = 3,    /* 到位并停止 PWM。 */
    LIFT_FAULT = 4,        /* 异常停机，查看 fault。 */
} lift_state_t;

typedef enum {
    LIFT_FAULT_NONE = 0,
    LIFT_FAULT_NO_PROGRESS = 1, /* 驱动中长时间没有朝目标移动。 */
    LIFT_FAULT_DIRECTION = 2,   /* 编码器显示丝杆持续向反方向移动。 */
    LIFT_FAULT_RANGE = 3,       /* 计数超出软件行程。 */
    LIFT_FAULT_TIMEOUT = 4,     /* 本次定位超过总时限。 */
    LIFT_FAULT_ESTOP = 5,
    LIFT_FAULT_LINK_LOST = 6,
    LIFT_FAULT_OVERSHOOT = 7,   /* 越过目标，超出到位容差。 */
} lift_fault_t;

typedef enum {
    LIFT_COMMAND_OK = 0,
    LIFT_COMMAND_BUSY = 1,
    LIFT_COMMAND_INVALID = 2,
    LIFT_COMMAND_UNCONFIGURED = 3,
    LIFT_COMMAND_NEEDS_ZERO = 4,
    LIFT_COMMAND_ESTOP = 5,
} lift_command_result_t;

typedef struct {
    float position_mm; /* 有零点时的相对高度。 */
    uint16_t target_mm; /* 最近一次接受的目标高度。 */
    int32_t encoder_count; /* 上电以来的原始有符号计数，便于标定。 */
    lift_state_t state;
    lift_fault_t fault;
    bool configured;
    bool referenced;
} lift_status_t;

/** 启动丝杆 PWM。返回硬件是否初始化成功。 */
bool app_lift_init(void);

/** 设置 counts_per_mm（A 相上升沿数/毫米）和软件行程 max_height_mm。
 *  返回是否接受；接受后需要重新设零。标定保存在 RAM 中。 */
lift_command_result_t app_lift_configure(float counts_per_mm, uint16_t max_height_mm);

/** 将当前计数记作零点。返回是否接受。 */
lift_command_result_t app_lift_set_zero(void);

/** 提交绝对高度 target_mm，从 now_ms（毫秒）开始计时。返回是否接受。 */
lift_command_result_t app_lift_move(uint16_t target_mm, uint32_t now_ms);

/** 停止当前运动。reason 为 NONE 时表示正常停止，否则上报对应故障。 */
void app_lift_stop(lift_fault_t reason);

/** 用当前时间 now_ms（毫秒）推进定位。主循环周期调用。 */
void app_lift_update(uint32_t now_ms);

/** 返回当前状态快照，可用于 UART1 上报。 */
lift_status_t app_lift_get_status(void);

#endif
