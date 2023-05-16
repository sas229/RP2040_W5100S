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

# Add required subdirectories to build examples.
add_subdirectory(libraries/picolog)
add_subdirectory(libraries/W5100S_arch)
add_subdirectory(libraries/W5100S_driver)
add_subdirectory(examples/poll)