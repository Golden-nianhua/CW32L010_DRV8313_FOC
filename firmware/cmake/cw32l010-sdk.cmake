set(CW32_SPL_ROOT "D:/Code/MCU/CW32/CW32L010_StandardPeripheralLib_V1.0.9"
    CACHE PATH "CW32L010 Standard Peripheral Library V1.0.9 root")
set(cw32_spl "${CW32_SPL_ROOT}/Libraries")
if(NOT EXISTS "${cw32_spl}/inc/cw32l010.h" OR NOT EXISTS "${cw32_spl}/src/cw32l010_gpio.c")
    message(FATAL_ERROR "Set CW32_SPL_ROOT to the CW32L010 Standard Peripheral Library V1.0.9 root.")
endif()

add_library(cw32_spl OBJECT)
foreach(driver adc atim btim flash gpio gtim i2c sysctrl systick)
    target_sources(cw32_spl PRIVATE "${cw32_spl}/src/cw32l010_${driver}.c")
endforeach()
target_include_directories(cw32_spl PUBLIC
    "${cw32_spl}/inc"
    "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Core/Include"
)
target_compile_definitions(cw32_spl PUBLIC USE_SPL_DRIVER CW32L010F8)
