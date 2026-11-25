cmake_minimum_required(VERSION 3.22)
project(chassis_host_tests C)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
enable_testing()

set(chassis_test_sources
    ../app/Src/chassis.c
    ../app/Src/app_motor.c
    ../app/Src/motor_speed.c
    ../app/Src/pid.c
    ../bsp/Src/bsp_motor.c
    ../bsp/Src/bsp_motor_encoder.c)

add_executable(chassis_host_tests test_chassis.c ${chassis_test_sources})
target_include_directories(chassis_host_tests PRIVATE stubs ../app/Inc ../bsp/Inc)
target_compile_options(chassis_host_tests PRIVATE -Wall -Wextra -Werror -Wmissing-prototypes -UNDEBUG)
target_link_libraries(chassis_host_tests PRIVATE m)
add_test(NAME chassis_control_regressions COMMAND chassis_host_tests)

# 循迹测试链接真实应用和 BSP，只有 GPIO、定时器寄存器由主机替身提供。
foreach(line_variant IN ITEMS standard high_reverse)
    set(line_target line_follow_${line_variant}_tests)
    add_executable(${line_target}
        test_line_follow.c
        ../app/Src/app_line_follow.c
        ../bsp/Src/bsp_line_sensor.c
        ${chassis_test_sources})
    target_include_directories(${line_target} PRIVATE stubs ../app/Inc ../bsp/Inc)
    target_compile_options(${line_target} PRIVATE -Wall -Wextra -Werror -Wmissing-prototypes -UNDEBUG)
    target_link_libraries(${line_target} PRIVATE m)
    target_compile_definitions(${line_target} PRIVATE
        "LINE_FOLLOW_STEERING_PID_GAINS={.kp=LINE_FOLLOW_STEERING_KP_RAD_S,.ki=0.0f,.kd=0.0f}")
    if(line_variant STREQUAL "high_reverse")
        target_compile_definitions(${line_target} PRIVATE
            LINE_FOLLOW_ACTIVE_LOW=0 LINE_FOLLOW_REVERSE_SENSOR_ORDER=1)
    endif()
    add_test(NAME ${line_target} COMMAND ${line_target})
endforeach()

add_executable(pid_control_tests
    test_pid_control.c
    ../app/Src/app_line_follow.c
    ../bsp/Src/bsp_line_sensor.c
    ${chassis_test_sources})
target_include_directories(pid_control_tests PRIVATE stubs ../app/Inc ../bsp/Inc)
target_compile_options(pid_control_tests PRIVATE -Wall -Wextra -Werror -Wmissing-prototypes -UNDEBUG)
target_compile_definitions(pid_control_tests PRIVATE
    "MOTOR_LEFT_SPEED_PID_GAINS={.kp=2.0f,.ki=4.0f,.kd=0.5f}"
    "MOTOR_RIGHT_SPEED_PID_GAINS={.kp=1.0f,.ki=2.0f,.kd=0.0f}"
    "LINE_FOLLOW_STEERING_PID_GAINS={.kp=0.2f,.ki=1.0f,.kd=0.06f}")
target_link_libraries(pid_control_tests PRIVATE m)
add_test(NAME pid_control_tests COMMAND pid_control_tests)
