cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include($ENV{FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

project(keyword_control C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(keyword_control 
    main.c
)

target_include_directories(keyword_control PRIVATE 
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(keyword_control 
    pico_stdlib 
    hardware_adc 
    hardware_timer
    hardware_pwm
    FreeRTOS-Kernel 
    FreeRTOS-Kernel-Heap4
)

# IMPORTANTE: Abilita USB CDC
pico_enable_stdio_usb(keyword_control 1)
pico_enable_stdio_uart(keyword_control 0)

pico_add_extra_outputs(keyword_control)

