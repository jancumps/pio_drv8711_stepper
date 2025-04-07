cmake_minimum_required(VERSION 3.28)

include(pico_sdk_import.cmake)

project(pio_drv8711_stepper C CXX ASM)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmodules-ts -fcommon")

pico_sdk_init()

add_executable(${CMAKE_PROJECT_NAME})
target_sources(${CMAKE_PROJECT_NAME}
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/source/main.cpp
)
target_sources(${CMAKE_PROJECT_NAME}
        PUBLIC
        FILE_SET cxx_modules TYPE CXX_MODULES FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/source/drv8711/drv8711.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/drv8711/drv8711_config.cpp
)

pico_generate_pio_header(${CMAKE_PROJECT_NAME}
        ${CMAKE_CURRENT_SOURCE_DIR}/source/stepper.pio
)

target_link_libraries( ${CMAKE_PROJECT_NAME}
        pico_stdlib
        hardware_gpio
        hardware_spi
        hardware_pio
)

