# Generated cmake Pico project file.
cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Directories.
set(BUILD "${CMAKE_CURRENT_SOURCE_DIR}/build")

# Pull in Raspberry Pi Pico SDK (must be before project).
include(${CMAKE_CURRENT_SOURCE_DIR}/libraries/pico-sdk/pico_sdk_init.cmake)

# Declare project.
project(W5100S C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Create W5100S interface library.
add_library(W5100S INTERFACE)
target_sources(W5100S PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/src/W5100S.c
    ${CMAKE_CURRENT_LIST_DIR}/libraries/ioLibrary_Driver/Ethernet/socket.c
    ${CMAKE_CURRENT_LIST_DIR}/libraries/ioLibrary_Driver/Ethernet/wizchip_conf.c
    ${CMAKE_CURRENT_LIST_DIR}/libraries/ioLibrary_Driver/Ethernet/W5100S/w5100s.c
    )
target_include_directories(W5100S INTERFACE 
    ${CMAKE_CURRENT_LIST_DIR}/src/
    ${CMAKE_CURRENT_LIST_DIR}/libraries/ioLibrary_Driver/Ethernet
    ${CMAKE_CURRENT_LIST_DIR}/libraries/ioLibrary_Driver/Ethernet/W5100S
    )
target_link_libraries(W5100S INTERFACE
    pico_stdlib
    pico_time
    pico_async_context_poll
    hardware_spi 
    hardware_dma
    hardware_clocks
    picolog 
    pico_lwip
    pico_lwip_nosys
    )
target_compile_definitions(W5100S INTERFACE 
    _WIZCHIP_=W5100S
    NOSYS=1
    )
message("W5100S interface library available.")

# Add required subdirectories to build examples.
add_subdirectory(libraries/picolog)
add_subdirectory(examples)