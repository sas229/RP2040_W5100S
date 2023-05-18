add_library(W5100S_driver INTERFACE)
target_sources(W5100S_driver INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/W5100S_driver.c
        ${CMAKE_CURRENT_LIST_DIR}/W5100S_lwip.c
        ${CMAKE_CURRENT_LIST_DIR}/../ioLibrary_Driver/Ethernet/socket.c
        ${CMAKE_CURRENT_LIST_DIR}/../ioLibrary_Driver/Ethernet/wizchip_conf.c
        ${CMAKE_CURRENT_LIST_DIR}/../ioLibrary_Driver/Ethernet/W5100S/w5100s.c
        )

target_include_directories(W5100S_driver INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${CMAKE_CURRENT_LIST_DIR}/../ioLibrary_Driver/Ethernet
        ${CMAKE_CURRENT_LIST_DIR}/../ioLibrary_Driver/Ethernet/W5100S
        )

target_link_libraries(W5100S_driver INTERFACE
        picolog
        pico_unique_id
        pico_async_context_poll
        hardware_spi 
        hardware_dma
        hardware_clocks
        )
target_compile_definitions(W5100S_driver INTERFACE
        _WIZCHIP_=W5100S
        )       
message("W5100S_driver interface library available.")