cmake_minimum_required(VERSION 3.28)

include(pico_sdk_import.cmake)

project(pio_drv8711_stepper C CXX ASM)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmodules-ts -fcommon -fno-rtti -fno-exceptions")

pico_sdk_init()

include(FetchContent)

FetchContent_Declare(stepper
  GIT_REPOSITORY "https://github.com/jancumps/pio_stepper_lib.git"
  GIT_TAG "origin/main"
)
FetchContent_MakeAvailable(stepper)

# for drv8711

FetchContent_Declare(drv8711
  GIT_REPOSITORY "https://github.com/jancumps/drv8711_lib.git"
  GIT_TAG "origin/main"
  # prevent makefile execution (if any)
  SOURCE_SUBDIR =
)
FetchContent_MakeAvailable(drv8711)

FetchContent_Declare(pico_drv8711
  GIT_REPOSITORY "https://github.com/jancumps/pico_drv8711_lib.git"
  GIT_TAG "origin/main"
  # prevent makefile execution (if any)
  SOURCE_SUBDIR =
)
FetchContent_MakeAvailable(pico_drv8711)

# end drv8711

add_executable(${CMAKE_PROJECT_NAME})
target_sources(${CMAKE_PROJECT_NAME}
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/source/main.cpp
)
target_sources(${CMAKE_PROJECT_NAME}
        PUBLIC
        FILE_SET cxx_modules TYPE CXX_MODULES FILES
)

# for drv8711
add_library(pico_drv8711)
target_sources(pico_drv8711
        PUBLIC
        FILE_SET cxx_modules TYPE CXX_MODULES FILES
        ${pico_drv8711_SOURCE_DIR}/source/drv8711_pico.cpp
        ${drv8711_SOURCE_DIR}/source/drv8711.cpp
        ${drv8711_SOURCE_DIR}/source/drv8711_config.cpp
)
target_link_libraries(pico_drv8711 $<COMPILE_ONLY:hardware_spi>)
# end drv8711

target_link_libraries( ${CMAKE_PROJECT_NAME}
        pico_stdlib
        hardware_gpio
        # for drv8711
        hardware_spi
        pico_drv8711
        # end drv8711
        stepper
)

