cmake_minimum_required(VERSION 3.22)

if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/cmake/arm-gcc.cmake")
endif()
if(NOT DEFINED CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type")
endif()

project(CW32L010_DRV8313_FOC LANGUAGES C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(
    "$<$<COMPILE_LANGUAGE:C>:-Wall;-Wextra;-Wpedantic;-ffunction-sections;-fdata-sections>"
    # CW32L010 has a 32-cycle multiplier. Let GCC trade Flash instructions for
    # shifts/additions when multiplying by constants.
    "$<$<COMPILE_LANGUAGE:C>:-mtune=cortex-m0plus.small-multiply>"
)

include(cmake/cw32l010-sdk.cmake)

add_library(motor_control STATIC
    Control/Src/foc.c
    Control/Src/hfi.c
    Control/Src/motion.c
    Control/Src/smo.c
    Control/Src/sensorless.c
)
target_include_directories(motor_control PUBLIC Control/Inc)
set_source_files_properties(Control/Src/foc.c Control/Src/hfi.c Control/Src/motion.c
    Control/Src/smo.c Control/Src/sensorless.c PROPERTIES COMPILE_OPTIONS "-O2")
set_source_files_properties(App/Src/main.c PROPERTIES COMPILE_OPTIONS "-O2")

# Link the board and startup objects directly so their IRQ handlers override
# the weak aliases in the vector table.
add_library(cw32_board OBJECT
    BSP/Src/board_time.c
    BSP/Src/bus_sense.c
    BSP/Src/bsp.c
    BSP/Src/current_adc.c
    BSP/Src/drv8313.c
    BSP/Src/mt6701.c
)
target_include_directories(cw32_board PUBLIC BSP/Inc)
target_link_libraries(cw32_board PRIVATE cw32_spl)
set_source_files_properties(BSP/Src/drv8313.c
    PROPERTIES COMPILE_OPTIONS "-O2")
set_source_files_properties(BSP/Src/current_adc.c
    PROPERTIES COMPILE_OPTIONS "-O3")

add_library(cw32_core OBJECT
    Core/Src/rtt.c
    Core/Src/system_cw32l010.c
    Core/startup_cw32l010.s
)
target_include_directories(cw32_core PUBLIC Core/Inc)
target_link_libraries(cw32_core PRIVATE cw32_spl)

add_executable(${PROJECT_NAME}
    App/Src/main.c
    App/Src/console.c
    $<TARGET_OBJECTS:cw32_spl>
)
target_include_directories(${PROJECT_NAME} PRIVATE App/Inc)
target_link_libraries(${PROJECT_NAME} PRIVATE motor_control cw32_board cw32_core)

set(linker_script "${CMAKE_CURRENT_SOURCE_DIR}/Core/CW32L010.ld")
set_target_properties(${PROJECT_NAME} PROPERTIES
    SUFFIX ".elf"
    LINK_DEPENDS "${linker_script}"
    ADDITIONAL_CLEAN_FILES "${PROJECT_NAME}.map;${PROJECT_NAME}.hex;${PROJECT_NAME}.bin"
)
target_link_options(${PROJECT_NAME} PRIVATE
    "-T${linker_script}"
    --specs=nano.specs
    -Wl,-Map=${PROJECT_NAME}.map,--gc-sections,--print-memory-usage
    -Wl,--no-warn-rwx-segments
)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND "${CMAKE_OBJCOPY}" -O ihex "$<TARGET_FILE:${PROJECT_NAME}>" "${PROJECT_NAME}.hex"
    COMMAND "${CMAKE_OBJCOPY}" -O binary "$<TARGET_FILE:${PROJECT_NAME}>" "${PROJECT_NAME}.bin"
    COMMENT "Generating HEX and BIN"
    VERBATIM
)

include(cmake/pyocd.cmake)
include(cmake/clion.cmake)

# Separate ARM image for offline execution of the observer without retaining
# unused SMO sections in the motor firmware.
add_executable(smo_validation Control/Src/smo.c Control/Src/foc.c)
target_include_directories(smo_validation PRIVATE Control/Inc)
set_target_properties(smo_validation PROPERTIES SUFFIX ".elf" LINK_DEPENDS "${linker_script}")
target_link_options(smo_validation PRIVATE
    "-T${linker_script}" -nostartfiles --specs=nano.specs
    -Wl,-e,Smo_Update,--no-warn-rwx-segments
)
