set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(ARM_GNU_TOOLCHAIN_ROOT "" CACHE PATH "Optional Arm GNU toolchain installation directory")
list(APPEND CMAKE_TRY_COMPILE_PLATFORM_VARIABLES ARM_GNU_TOOLCHAIN_ROOT)

if(ARM_GNU_TOOLCHAIN_ROOT)
    set(arm_bin "${ARM_GNU_TOOLCHAIN_ROOT}/bin")
    set(arm_search PATHS "${arm_bin}" NO_DEFAULT_PATH)
endif()
find_program(CMAKE_C_COMPILER NAMES arm-none-eabi-gcc ${arm_search} REQUIRED)
set(CMAKE_ASM_COMPILER "${CMAKE_C_COMPILER}")

get_filename_component(arm_bin "${CMAKE_C_COMPILER}" DIRECTORY)
find_program(CMAKE_OBJCOPY NAMES arm-none-eabi-objcopy PATHS "${arm_bin}" NO_DEFAULT_PATH REQUIRED)
find_program(CMAKE_SIZE NAMES arm-none-eabi-size PATHS "${arm_bin}" NO_DEFAULT_PATH REQUIRED)

set(CMAKE_C_FLAGS_INIT "-mcpu=cortex-m0plus -mthumb")
set(CMAKE_ASM_FLAGS_INIT "${CMAKE_C_FLAGS_INIT}")
set(CMAKE_C_FLAGS_DEBUG "-O0 -g3" CACHE STRING "C compiler flags for Debug")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0" CACHE STRING "C compiler flags for Release")
