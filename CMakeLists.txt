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
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

# Set the project name
set(CMAKE_PROJECT_NAME momentum)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Create additional executable object types (hex/bin)
find_program(CMAKE_OBJCOPY arm-none-eabi-objcopy)
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user sources here
        Core/BMP3_SensorAPI/bmp3.c
        Core/momentum_driver/momentum_can_dbc.c
        Core/momentum_driver/momentum_can_driver.c
        Core/momentum_driver/momentum_spi_driver.c
        Core/sh2/euler.c
        Core/sh2/sh2.c
        Core/sh2/sh2_SensorValue.c
        Core/sh2/sh2_util.c
        Core/sh2/shtp.c
        Core/Src/bmp3_hal_i2c.c
        Core/Src/bmp390_runner.c
        Core/Src/bno085_runner.c
        Core/Src/callbacks.c
        Core/Src/can.c
        Core/Src/comm.c
        Core/Src/init.c
        Core/Src/momentum_runner.c
        Core/Src/rtc.c
        Core/Src/run.c
        Core/Src/scheduler.c
        Core/Src/sh2_hal_spi.c
        Core/Src/telemetry.c
        Core/Src/ublox_hal_uart.c
        Core/Src/ws2812b_hal_pwm.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined include paths
        Core/BMP3_SensorAPI
        Core/momentum_driver
        Core/sh2
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx

        # Add user defined libraries
)
