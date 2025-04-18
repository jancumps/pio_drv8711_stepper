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
  SOURCE_SUBDIR =
)
FetchContent_MakeAvailable(stepper)
set(stepper_PATH ${stepper_SOURCE_DIR})

add_executable(${CMAKE_PROJECT_NAME})
target_sources(${CMAKE_PROJECT_NAME}
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/source/main.cpp
)
target_sources(${CMAKE_PROJECT_NAME}
        PUBLIC
        FILE_SET cxx_modules TYPE CXX_MODULES FILES
        # for drv8711
        ${CMAKE_CURRENT_SOURCE_DIR}/source/drv8711_driver/drv8711.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/drv8711_driver/drv8711_config.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/source/drv8711_pico.cpp
        # end drv8711
)

add_library(stepper_lib)
target_sources(stepper_lib
        PUBLIC
        ${stepper_PATH}/source/stepper_impl.cpp
)
target_sources(stepper_lib
        PUBLIC
        FILE_SET cxx_modules TYPE CXX_MODULES FILES
        ${stepper_PATH}/source/pio_irq_util.cpp
        ${stepper_PATH}/source/stepper.cpp
)
pico_generate_pio_header(stepper_lib
        ${stepper_PATH}/source/stepper.pio
)

target_link_libraries( stepper_lib
        pico_stdlib
        hardware_pio
        hardware_gpio
)

target_link_libraries( ${CMAKE_PROJECT_NAME}
        pico_stdlib
        hardware_pio
        hardware_gpio
        # for drv8711
        hardware_spi
        # end drv8711
        stepper_lib
)

