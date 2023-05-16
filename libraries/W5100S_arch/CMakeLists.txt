add_library(W5100S_arch_poll INTERFACE)
target_sources(W5100S_arch_poll INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/W5100S_arch.c
        ${CMAKE_CURRENT_LIST_DIR}/W5100S_arch_poll.c
        )

target_include_directories(W5100S_arch_poll INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/include/)

target_link_libraries(W5100S_arch_poll INTERFACE
        pico_unique_id
        W5100S_driver
        pico_lwip_nosys
        )
target_compile_definitions(W5100S_arch_poll INTERFACE
        W5100S_ARCH_POLL=1
        W5100S_LWIP=1
        NO_SYS=1
        )
message("W5100S_arch_poll interface library available.")

add_library(W5100S_arch_threadsafe_background INTERFACE)
target_sources(W5100S_arch_threadsafe_background INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/W5100S_arch.c
        ${CMAKE_CURRENT_LIST_DIR}/W5100S_arch_threadsafe_background.c
        )

target_include_directories(W5100S_arch_threadsafe_background INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/include)

target_link_libraries(W5100S_arch_threadsafe_background INTERFACE
        pico_unique_id
        W5100S_driver
        pico_lwip_nosys
        )
target_compile_definitions(W5100S_arch_threadsafe_background INTERFACE
        W5100S_ARCH_THREADSAFE_BACKGROUND=1
        W5100S_LWIP=1
        NO_SYS=1
        )
message("W5100S_arch_threadsafe_background interface library available.")

add_library(W5100S_arch_freertos INTERFACE)
target_sources(W5100S_arch_freertos INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/W5100S_arch.c
        ${CMAKE_CURRENT_LIST_DIR}/W5100S_arch_freertos.c
        )

target_include_directories(W5100S_arch_freertos INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/include)

target_link_libraries(W5100S_arch_freertos INTERFACE
        pico_unique_id
        W5100S_driver
        pico_lwip_freertos
        )
target_compile_definitions(W5100S_arch_freertos INTERFACE
        W5100S_ARCH_FREERTOS=1
        W5100S_LWIP=1
        NO_SYS=0
        )
message("W5100S_arch_freertos interface library available.")