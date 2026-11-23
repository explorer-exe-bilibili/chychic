#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H

/** motor_id_t：电机编号；LEFT/RIGHT 分别表示车辆前进方向的左轮和右轮。 */
typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT = 1,
    MOTOR_COUNT = 2 //电机总数
} motor_id_t;

#endif
