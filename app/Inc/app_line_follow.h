#ifndef APP_LINE_FOLLOW_H
#define APP_LINE_FOLLOW_H

#include <stdbool.h>
#include <stdint.h>

/** line_follow_state_t：循迹当前状态，用于调试识别结果和停车原因。 */
typedef enum {
    LINE_FOLLOW_DISABLED = 0, /* 未启动；周期采样继续，但不覆盖手动底盘指令。 */
    LINE_FOLLOW_ACQUIRING,    /* 等待稳定的正常线型，暂不行驶。 */
    LINE_FOLLOW_TRACKING,     /* 正常循迹，更新速度及转向目标。 */
    LINE_FOLLOW_LOST,         /* 五路均未识别黑线，清零底盘指令。 */
    LINE_FOLLOW_WIDE_LINE,    /* 连续黑线过宽，可能是路口，暂不自行选路。 */
    LINE_FOLLOW_AMBIGUOUS     /* 同时出现多个不相邻黑线段，暂时停车。 */
} line_follow_state_t;

/** line_follow_status_t：循迹缓存快照；查询返回副本，不触发新的 GPIO 读取。 */
typedef struct {
    bool enabled;                /* 是否已通过 start 启动。 */
    line_follow_state_t state;   /* 最近一次更新所处的工作状态。 */
    uint8_t raw_levels;          /* 原始电平：bit0～bit4 对应 OUT1～OUT5，1 为高。 */
    uint8_t line_mask;           /* 归一化黑线位图：bit0 最左、bit4 最右，1 为黑线。 */
    uint8_t active_sensor_count; /* 本次检测到黑线的探头数量。 */
    float position_error;        /* 最新有效位置偏差，左正右负，范围 [-2, 2]。 */
    float linear_speed_mm_s;     /* 最近下发的车体线速度目标，毫米/秒。 */
    float angular_speed_rad_s;   /* 最近下发的车体角速度目标，弧度/秒，左转为正。 */
} line_follow_status_t;

/**
 * app_line_follow_init：清零循迹状态，默认不启动，并请求底盘停住。
 * 参数：无。返回：无。应在 chassis_init 成功后调用。
 */
void app_line_follow_init(void);

/**
 * app_line_follow_start：清除旧指令，启动循迹，等待稳定线型再运动。
 * 参数：无。返回：无。调用一次即可；丢线等状态恢复为有效线型后会自动重新捕获。
 */
void app_line_follow_start(void);

/**
 * app_line_follow_stop：退出循迹并清零底盘指令。
 * 参数：无。返回：无。退出后可重新使用手动底盘指令。
 */
void app_line_follow_stop(void);

/**
 * app_line_follow_update：轮询五路输入，生成底盘线速度和角速度目标。
 * dt_ms：本次控制间隔，毫秒，0 时不采样也不更新。返回：无。
 * 在同一主循环先调用本函数，再且仅再调用一次 chassis_update(dt_ms)。
 * 未启动时只更新传感器快照；工作中连续有效新线型确认前保持上一条循迹目标，
 * 丢线、宽线、多线段均立即清零指令；恢复后重新等待稳定样本。
 */
void app_line_follow_update(uint32_t dt_ms);

/**
 * app_line_follow_get_status：获取调试用缓存快照。
 * status：接收状态的非空指针。返回：成功为 true，空指针返回 false。
 * 所有接口由同一主循环上下文调用；不承诺跨中断的原子快照。
 */
bool app_line_follow_get_status(line_follow_status_t *status);

#endif
