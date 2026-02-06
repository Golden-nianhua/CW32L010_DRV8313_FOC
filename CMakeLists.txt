cmake_minimum_required(VERSION 3.22)

set(ENV{PATH} "$ENV{PATH};D:/Code/C envs/arm-gnu-toolchain/arm-gnu-toolchain-14.2.rel1-mingw-w64-x86_64-arm-none-eabi/bin")

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
set(CMAKE_PROJECT_NAME CW32L010_DRV8313_FOC)

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
add_custom_command(
        TARGET ${CMAKE_PROJECT_NAME}
        POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} ${CMAKE_PROJECT_NAME}.elf -Oihex ${CMAKE_PROJECT_NAME}.hex
        COMMAND ${CMAKE_OBJCOPY} ${CMAKE_PROJECT_NAME}.elf -Obinary ${CMAKE_PROJECT_NAME}.bin
        COMMENT "Building hex&bin files"
)


# Add sources
add_subdirectory(cmake/cw32)

set(CMSISCORE ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Core)
#add_subdirectory(Drivers/CMSIS-DSP/)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user sources here
        ${CMAKE_SOURCE_DIR}/Core/Src/bsp.c
        ${CMAKE_SOURCE_DIR}/Core/Src/foc.c
        ${CMAKE_SOURCE_DIR}/Core/Src/pid.c
        ${CMAKE_SOURCE_DIR}/Core/Src/filter.c
        ${CMAKE_SOURCE_DIR}/Core/Src/position_sensor.c
        ${CMAKE_SOURCE_DIR}/Core/Src/current.c
        ${CMAKE_SOURCE_DIR}/Hardware/Src/mt6701.c
        ${CMAKE_SOURCE_DIR}/Hardware/Src/drv8313.c
        ${CMAKE_SOURCE_DIR}/Hardware/Src/led.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined include paths
        ${CMAKE_SOURCE_DIR}/Hardware/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
        cw32
#        CMSISDSP
        # Add user defined libraries
)

set(PYOCD_CONENT "target_override: CW32L010F8\npack: Misc/WHXY.CW32L010_DFP.1.0.0.pack\nfrequency: 10000000")
file(WRITE Misc/pyocd.yaml ${PYOCD_CONENT})