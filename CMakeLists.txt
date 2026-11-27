cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME chychic_project)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        app/Src/app_motor.c
        app/Inc/app_motor.h
        app/Src/pid.c
        app/Inc/pid.h
        bsp/Src/bsp_motor.c
        bsp/Inc/bsp_motor.h
        app/Src/motor_speed.c
        app/Inc/motor_speed.h
        bsp/Src/bsp_motor_encoder.c
        bsp/Inc/bsp_motor_encoder.h
        app/Src/chassis.c
        app/Inc/chassis.h
        app/Inc/chassis_config.h
        bsp/Inc/motor_types.h
        bsp/Src/gettick.c
        bsp/Inc/gettick.h
        bsp/Src/bsp_line_sensor.c
        bsp/Inc/bsp_line_sensor.h
        app/Src/app_line_follow.c
        app/Inc/app_line_follow.h
        app/Inc/line_follow_config.h
        app/Src/app_lift.c
        bsp/Src/bsp_lift.c
        app/Src/app_comm.c
        app/Src/app_comm_text.c
        app/Inc/app_comm.h
        app/Src/protocol.c
        app/Inc/protocol.h)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        ${CMAKE_CURRENT_SOURCE_DIR}/app/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/bsp/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Enable float formatting in newlib-nano for the UART plot and PID readback.
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE "LINKER:-u,_printf_float")

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
