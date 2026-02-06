cmake_minimum_required(VERSION 3.22)
# Enable CMake support for ASM and C languages
enable_language(C ASM)
# symbols (macros)
set(CW_Defines_Syms
        USE_SPL_DRIVER
        CW32L010F8
        #$<$<CONFIG:Debug>:DEBUG>
)

# include paths
set(CW_Include_Dirs
        ${CMAKE_SOURCE_DIR}/Core/Inc
        ${CMAKE_SOURCE_DIR}/Drivers/CW32L010_SPI_Driver/inc
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/CW/CW32L010/Include
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Core/Include
)

# application sources
set(CW_Application_Src
        ${CMAKE_SOURCE_DIR}/Core/Src/main.c
        ${CMAKE_SOURCE_DIR}/Core/Src/interrupts_cw32l010.c
		${CMAKE_SOURCE_DIR}/Core/Src/sysmem.c
		${CMAKE_SOURCE_DIR}/Core/Src/syscalls.c
        ${CMAKE_SOURCE_DIR}/startup_cw32l010.s
)

# DDL Drivers
set(CW32_Drivers_Src
        ${CMAKE_SOURCE_DIR}/Core/Src/system_cw32l010.c
        ${CMAKE_SOURCE_DIR}/Drivers/CW32L010_SPI_Driver/src/cw32l010_adc.c
        ${CMAKE_SOURCE_DIR}/Drivers/CW32L010_SPI_Driver/src/cw32l010_atim.c
        ${CMAKE_SOURCE_DIR}/Drivers/CW32L010_SPI_Driver/src/cw32l010_btim.c
        ${CMAKE_SOURCE_DIR}/Drivers/CW32L010_SPI_Driver/src/cw32l010_crc.c
        ${CMAKE_SOURCE_DIR}/Drivers/CW32L010_SPI_Driver/src/cw32l010_debug.c
        ${CMAKE_SOURCE_DIR}/Drivers/CW32L010_SPI_Driver/src/cw32l010_digitalsign.c
        ${CMAKE_SOURCE_DIR}/Drivers/CW32L010_SPI_Driver/src/cw32l010_flash.c
        ${CMAKE_SOURCE_DIR}/Drivers/CW32L010_SPI_Driver/src/cw32l010_gpio.c
        ${CMAKE_SOURCE_DIR}/Drivers/CW32L010_SPI_Driver/src/cw32l010_gtim.c
        ${CMAKE_SOURCE_DIR}/Drivers/CW32L010_SPI_Driver/src/cw32l010_i2c.c
        ${CMAKE_SOURCE_DIR}/Drivers/CW32L010_SPI_Driver/src/cw32l010_iwdt.c
        ${CMAKE_SOURCE_DIR}/Drivers/CW32L010_SPI_Driver/src/cw32l010_lptim.c
        ${CMAKE_SOURCE_DIR}/Drivers/CW32L010_SPI_Driver/src/cw32l010_lvd.c
        ${CMAKE_SOURCE_DIR}/Drivers/CW32L010_SPI_Driver/src/cw32l010_pwr.c
        ${CMAKE_SOURCE_DIR}/Drivers/CW32L010_SPI_Driver/src/cw32l010_ram.c
        ${CMAKE_SOURCE_DIR}/Drivers/CW32L010_SPI_Driver/src/cw32l010_rtc.c
        ${CMAKE_SOURCE_DIR}/Drivers/CW32L010_SPI_Driver/src/cw32l010_spi.c
        ${CMAKE_SOURCE_DIR}/Drivers/CW32L010_SPI_Driver/src/cw32l010_sysctrl.c
        ${CMAKE_SOURCE_DIR}/Drivers/CW32L010_SPI_Driver/src/cw32l010_systick.c
        ${CMAKE_SOURCE_DIR}/Drivers/CW32L010_SPI_Driver/src/cw32l010_uart.c
        ${CMAKE_SOURCE_DIR}/Drivers/CW32L010_SPI_Driver/src/cw32l010_vc.c
        ${CMAKE_SOURCE_DIR}/Drivers/CW32L010_SPI_Driver/src/i2c_eeprom.c
)

# Drivers Midllewares


# Link directories setup
set(CW_LINK_DIRS

)
# Project static libraries
set(CW_LINK_LIBS
        CW32_Drivers

)
# Interface library for includes and symbols
add_library(cw32 INTERFACE)
target_include_directories(cw32 INTERFACE ${CW_Include_Dirs})
target_compile_definitions(cw32 INTERFACE ${CW_Defines_Syms})

# Create CW32_Drivers static library
add_library(CW32_Drivers OBJECT)
target_sources(CW32_Drivers PRIVATE ${CW32_Drivers_Src})
target_link_libraries(CW32_Drivers PUBLIC cw32)


# Add application sources to the project
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${CW_Application_Src})

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CW_LINK_DIRS})

# Add libraries to the project
target_link_libraries(${CMAKE_PROJECT_NAME} ${CW_LINK_LIBS})

# Add the map file to the list of files to be removed with 'clean' target
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map)

# Validate thatcode is compatible with C standard
if ((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif ()
