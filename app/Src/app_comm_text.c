#include "app_comm_text.h"
#include "protocol.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

bool app_comm_text_feed(comm_text_rx_t *rx, uint8_t byte, bool at_frame_boundary, char **line) {
    *line = NULL;
    if (byte == PROTO_SOF1) {
        /* 允许二进制帧打断一条漏了结束符的文本命令。 */
        rx->receiving = false;
        return false;
    }
    if (byte == '!' && (at_frame_boundary || rx->receiving)) {
        rx->length = 0;
        rx->discard = false;
        rx->receiving = true;
        return true;
    }
    if (!rx->receiving) {
        return false;
    }

    if (byte == '#' || byte == '\r' || byte == '\n') {
        rx->receiving = false;
        if (!rx->discard) {
            rx->line[rx->length] = '\0';
            *line = rx->line;
        }
        return true;
    }
    if (rx->discard || rx->length >= sizeof(rx->line) - 1U ||
        ((byte < ' ' || byte > '~') && byte != '\t')) {
        rx->discard = true;
        return true;
    }
    rx->line[rx->length++] = (char)byte;
    return true;
}

/** token 为单个字段；value 接收有限浮点数。返回是否完整转换成功。 */
static bool parse_number(const char *token, float *value) {
    char *end;
    *value = strtof(token, &end);
    return end != token && *end == '\0' && isfinite(*value);
}

comm_text_command_t app_comm_text_parse(char *line) {
    comm_text_command_t command = {0};
    char *tokens[5];
    size_t count = 0;
    char *token = strtok(line, " \t");
    while (token != NULL) {
        if (count == sizeof(tokens) / sizeof(tokens[0])) {
            return command;
        }
        tokens[count++] = token;
        token = strtok(NULL, " \t");
    }
    if (count == 0U) {
        return command;
    }

    if (count == 1U) {
        if (strcmp(tokens[0], "idle") == 0) {
            command.action = COMM_TEXT_IDLE;
        } else if (strcmp(tokens[0], "estop") == 0) {
            command.action = COMM_TEXT_ESTOP;
        } else if (strcmp(tokens[0], "ping") == 0) {
            command.action = COMM_TEXT_PING;
        }
    } else if (count == 2U && strcmp(tokens[0], "plot") == 0) {
        if (strcmp(tokens[1], "on") == 0) {
            command.action = COMM_TEXT_PLOT_ON;
        } else if (strcmp(tokens[1], "off") == 0) {
            command.action = COMM_TEXT_PLOT_OFF;
        }
    } else if (count == 2U && strcmp(tokens[0], "speed") == 0) {
        if (parse_number(tokens[1], &command.speed_mm_s)) {
            command.action = COMM_TEXT_SPEED;
        }
    } else if (count == 2U && strcmp(tokens[0], "pid") == 0 && strcmp(tokens[1], "get") == 0) {
        command.action = COMM_TEXT_PID_GET;
    } else if (count == 5U && strcmp(tokens[0], "pid") == 0) {
        if (strcmp(tokens[1], "left") == 0) {
            command.wheel = MOTOR_LEFT;
        } else if (strcmp(tokens[1], "right") == 0) {
            command.wheel = MOTOR_RIGHT;
        } else if (strcmp(tokens[1], "both") == 0) {
            command.wheel = MOTOR_COUNT;
        } else {
            return command;
        }
        if (parse_number(tokens[2], &command.gains.kp) &&
            parse_number(tokens[3], &command.gains.ki) &&
            parse_number(tokens[4], &command.gains.kd)) {
            command.action = COMM_TEXT_PID_SET;
        }
    }
    return command;
}

size_t app_comm_text_plot(char *out, size_t capacity, float left_target, float left_actual,
                          float right_target, float right_actual) {
    int length = snprintf(out, capacity,
                          "left_target=%.2f,left_actual=%.2f,right_target=%.2f,right_actual=%.2f\n",
                          (double)left_target, (double)left_actual, (double)right_target,
                          (double)right_actual);
    return length > 0 && (size_t)length < capacity ? (size_t)length : 0U;
}
