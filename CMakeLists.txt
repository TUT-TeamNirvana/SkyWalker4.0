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
set(CMAKE_PROJECT_NAME SkyWalker4.0)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Core/application/BottomControl/BottomControl.c
        Core/application/BottomControl/BottomControl.h
        Core/modules/pid/pos_pid.h
        Core/modules/pid/pos_pid.c
        Core/modules/LK-MG/lk-mg_motor.c
        Core/modules/LK-MG/lk-mg_motor.h)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
        Core/bsp/can/bsp_can.c
        Core/bsp/sbus/sbus.c
        Core/bsp/uart6/bsp_uart6.c
        Core/bsp/uart8/bsp_uart8.c
        Core/bsp/log/bsp_log.c
        Core/bsp/log/printf.c

        Core/modules/pid/pid.c
        Core/modules/3508/m3508_motor.c
        Core/modules/LK-MG/lk-mg_motor.c

        Core/application/BottomControl/BottomControl.c

        Core/SEGGER_RTT/RTT/SEGGER_RTT.c
        Core/SEGGER_RTT/RTT/SEGGER_RTT_printf.c
        Core/SEGGER_RTT/RTT/SEGGER_RTT_ASM_ARMv7M.s
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        Core/bsp/can
        Core/bsp/sbus
        Core/bsp/uart6
        Core/bsp/uart8
        Core/bsp/log

        Core/modules/pid
        Core/modules/3508
        Core/modules/LK-MG

        Core/application/BottomControl

        Core/SEGGER_RTT/Config
        Core/SEGGER_RTT/RTT
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
