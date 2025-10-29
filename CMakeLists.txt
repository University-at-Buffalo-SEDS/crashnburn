cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME crashnburn)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})


# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

#set(SEDSPRINTF_STM_BUILD ON)
add_subdirectory(${CMAKE_SOURCE_DIR}/sedsprintf_rs)
target_link_libraries(${CMAKE_PROJECT_NAME} sedsprintf_rs)


# add_subdirectory(${CMAKE_SOURCE_DIR}/sedsprintf)
# target_link_libraries(${CMAKE_PROJECT_NAME} sedsprintf)
# Add linked libraries
target_link_libraries(
    stm32cubemx
)

# ---- FreeRTOS config interface (provides FreeRTOSConfig.h include path) ----
add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM INTERFACE
    ${CMAKE_SOURCE_DIR}/Core/Inc   # path containing FreeRTOSConfig.h
)

# ---- FreeRTOS-Kernel (select port/heap before adding subdir) ----
set(FREERTOS_PORT GCC_ARM_CM4F CACHE STRING "")
set(FREERTOS_HEAP 4             CACHE STRING "")  # heap_4.c
add_subdirectory(${CMAKE_SOURCE_DIR}/FreeRTOS-Kernel)

target_link_libraries(freertos_kernel PUBLIC freertos_config)
target_compile_options(freertos_kernel PRIVATE
    -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
)

target_link_libraries(${CMAKE_PROJECT_NAME} freertos_kernel)

# ---- CubeMX generated lib must link into crashnburn (PLAIN signature) ----
target_link_libraries(${CMAKE_PROJECT_NAME} stm32cubemx)