#ifndef APP_COMM_TEXT_H
#define APP_COMM_TEXT_H

#include "motor_types.h"
#include "pid.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    char line[80];     /* 文本命令缓冲，不含起止符。 */
    size_t length;     /* 已接收的字符数。 */
    bool receiving;    /* 在二进制帧外收到了文本起始符。 */
    bool discard;      /* 本条命令过长或含无效字符。 */
} comm_text_rx_t;

typedef enum {
    COMM_TEXT_INVALID = 0,
    COMM_TEXT_PLOT_ON,
    COMM_TEXT_PLOT_OFF,
    COMM_TEXT_PID_GET,
    COMM_TEXT_PID_SET,
    COMM_TEXT_SPEED,
    COMM_TEXT_IDLE,
    COMM_TEXT_ESTOP,
    COMM_TEXT_PING,
} comm_text_action_t;

typedef struct {
    comm_text_action_t action;
    motor_id_t wheel;  /* PID_SET 的轮编号，MOTOR_COUNT 表示同时设置双轮。 */
    pid_gains_t gains;  /* PID_SET 的三个增益。 */
    float speed_mm_s;  /* SPEED 指令的直行速度，毫米/秒。 */
} comm_text_command_t;

/**
 * 收一个字节。rx 为文本接收状态，at_frame_boundary 表示二进制解析器在等帧头。
 * line 为输出，命令完整时指向 NUL 结尾的缓冲。返回 true 表示该字节已由文本接收器消费。
 */
bool app_comm_text_feed(comm_text_rx_t *rx, uint8_t byte, bool at_frame_boundary, char **line);

/** 解析 line 中的命令。line 内容会被分词；返回动作及其参数。 */
comm_text_command_t app_comm_text_parse(char *line);

/**
 * 将左右轮的目标和实测速度（毫米/秒）格式化为一行。
 * out/capacity 为输出缓冲及容量。返回可发送的字节数，容量不足时返回 0。
 */
size_t app_comm_text_plot(char *out, size_t capacity, float left_target, float left_actual,
                          float right_target, float right_actual);

#endif
