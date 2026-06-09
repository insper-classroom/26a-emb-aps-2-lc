
if (NOT FREERTOS_KERNEL_PATH AND NOT DEFINED ENV{FREERTOS_KERNEL_PATH})
    message("Skipping FreeRTOS examples as FREERTOS_KERNEL_PATH not defined")
    return()
endif()

include(FreeRTOS_Kernel_import.cmake)

# Áudio (main.c) + heurística de gesto do IMU (gesture.cpp). Sem Edge Impulse.
add_executable(pico_emb
    main.c
    audio.c
    gesture.cpp
    )

target_include_directories(pico_emb PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    )

# Linking to FreeRTOS-Kernel-Heap4 means we use a dynamic heap for allocations
target_link_libraries(pico_emb
    pico_stdlib
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
    hardware_dma
    hardware_pio
    hardware_timer
    hardware_clocks
    hardware_pwm
    hardware_adc
    hardware_irq
    hardware_i2c
)

target_compile_definitions(pico_emb PRIVATE
    configNUMBER_OF_CORES=1
    # O áudio é BINÁRIO. Sem isto, o stdio traduz todo byte 0x0A em 0x0D 0x0A,
    # inserindo bytes no meio do stream e desalinhando as amostras.
    PICO_STDIO_DEFAULT_CRLF=0
    # UART a 460800 baud (~46 KB/s) cobre os 16 kHz x 16-bit (32 KB/s) com folga.
    PICO_DEFAULT_UART_BAUD_RATE=460800
    )

if(PICO_CYW43_SUPPORTED)
    # For led support on pico_w
    target_link_libraries(pico_emb PRIVATE
        pico_cyw43_arch_none
        )
endif()

# Transporte via USB CDC nativa — único com banda pros 16 kHz (32 KB/s).
pico_enable_stdio_usb(pico_emb 1)
pico_enable_stdio_uart(pico_emb 0)

pico_add_extra_outputs(pico_emb)

set_target_properties(pico_emb PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
