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
