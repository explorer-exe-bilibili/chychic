#ifndef LINE_FOLLOW_CONFIG_H
#define LINE_FOLLOW_CONFIG_H

/*
 * 黑白极性及安装顺序尚未实测，先按 HW-871 常见资料配置。
 * OUT1～OUT5 默认对应车体前进方向看过去的从左到右。
 * #ifndef 允许主机测试用编译参数覆盖这些配置；实际调参可直接修改默认值。
 */
#ifndef LINE_FOLLOW_ACTIVE_LOW
#define LINE_FOLLOW_ACTIVE_LOW 1 /* 1：低电平表示黑线；0：高电平表示黑线。 */
#endif
#ifndef LINE_FOLLOW_REVERSE_SENSOR_ORDER
#define LINE_FOLLOW_REVERSE_SENSOR_ORDER 0 /* OUT1 在最右侧时改为 1。 */
#endif

#ifndef LINE_FOLLOW_STABLE_SAMPLES
#define LINE_FOLLOW_STABLE_SAMPLES 3U /* 相同有效位图连续出现 3 次才采用，名义约 30ms。 */
#endif
#ifndef LINE_FOLLOW_MAX_ACTIVE_SENSORS
#define LINE_FOLLOW_MAX_ACTIVE_SENSORS 2U /* 连续 1～2 路视为正常；更宽的黑线暂时停车。 */
#endif
#ifndef LINE_FOLLOW_FORWARD_SPEED_MM_S
#define LINE_FOLLOW_FORWARD_SPEED_MM_S 120.0f /* 首版低速前进目标，毫米/秒，需实车标定。 */
#endif
#ifndef LINE_FOLLOW_STEERING_KP_RAD_S
#define LINE_FOLLOW_STEERING_KP_RAD_S 0.8f /* 每单位位置偏差对应的角速度，弧度/秒。 */
#endif
#ifndef LINE_FOLLOW_MAX_ANGULAR_SPEED_RAD_S
#define LINE_FOLLOW_MAX_ANGULAR_SPEED_RAD_S 1.2f /* 转向角速度限幅，弧度/秒。 */
#endif

#endif
