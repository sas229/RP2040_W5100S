set(TARGET_NAME W5100S_poll)
add_executable(${TARGET_NAME}
        ${TARGET_NAME}.c
        )
target_link_libraries(${TARGET_NAME} PRIVATE
        pico_stdlib
        picolog
        W5100S_arch_poll
        )
pico_enable_stdio_usb(${TARGET_NAME} 1)
pico_enable_stdio_uart(${TARGET_NAME} 0)
pico_add_extra_outputs(${TARGET_NAME})