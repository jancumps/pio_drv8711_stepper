cmake_minimum_required(VERSION 3.28)

include(pico_sdk_import.cmake)

project(pio_drv8711_stepper C CXX ASM)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmodules-ts -fcommon")

pico_sdk_init()

include(FetchContent)

FetchContent_Declare(stepper
  GIT_REPOSITORY "https://github.com/jancumps/pio_stepper_lib.git"
  GIT_TAG "origin/main"
)
FetchContent_MakeAvailable(stepper)

FetchContent_Declare(drv8711
  GIT_REPOSITORY "https://github.com/jancumps/drv8711_lib.git"
  GIT_TAG "origin/main"
  # prevent makefile execution (if any)
  SOURCE_SUBDIR =
)
FetchContent_MakeAvailable(drv8711)

add_executable(${CMAKE_PROJECT_NAME})
target_sources(${CMAKE_PROJECT_NAME}
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/source/main.cpp
)
target_sources(${CMAKE_PROJECT_NAME}
        PUBLIC
        FILE_SET cxx_modules TYPE CXX_MODULES FILES
        # for drv8711
        ${drv8711_SOURCE_DIR}/source/drv8711.cpp
        ${drv8711_SOURCE_DIR}/source//drv8711_config.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/drv8711_pico.cpp
        # end drv8711
)

target_link_libraries( ${CMAKE_PROJECT_NAME}
        pico_stdlib
        hardware_gpio
        # for drv8711
        hardware_spi
        # end drv8711
        stepper
)

