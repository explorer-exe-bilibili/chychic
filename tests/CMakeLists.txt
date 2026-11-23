cmake_minimum_required(VERSION 3.22)
project(chassis_host_tests C)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
enable_testing()

add_executable(chassis_host_tests
    test_chassis.c
    ../app/Src/chassis.c
    ../app/Src/app_motor.c
    ../app/Src/motor_speed.c
    ../bsp/Src/bsp_motor.c
    ../bsp/Src/bsp_motor_encoder.c)
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
        ../app/Src/chassis.c
        ../app/Src/app_motor.c
        ../app/Src/motor_speed.c
        ../bsp/Src/bsp_motor.c
        ../bsp/Src/bsp_motor_encoder.c)
    target_include_directories(${line_target} PRIVATE stubs ../app/Inc ../bsp/Inc)
    target_compile_options(${line_target} PRIVATE -Wall -Wextra -Werror -Wmissing-prototypes -UNDEBUG)
    target_link_libraries(${line_target} PRIVATE m)
    if(line_variant STREQUAL "high_reverse")
        target_compile_definitions(${line_target} PRIVATE
            LINE_FOLLOW_ACTIVE_LOW=0 LINE_FOLLOW_REVERSE_SENSOR_ORDER=1)
    endif()
    add_test(NAME ${line_target} COMMAND ${line_target})
endforeach()
