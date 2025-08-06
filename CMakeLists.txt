cmake_minimum_required(VERSION 3.28)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmodules-ts -fcommon -fno-rtti -fno-exceptions")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion RISCV_ZCB_RPI_2_1_1_3)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(pio_drv8711_stepper C CXX ASM)


pico_sdk_init()

include(FetchContent)

FetchContent_Declare(stepper
  GIT_REPOSITORY "https://github.com/jancumps/pio_stepper_lib.git"
  GIT_TAG "origin/main"
)
FetchContent_MakeAvailable(stepper)

# for drv8711

FetchContent_Declare(stepper_driver
  GIT_REPOSITORY "https://github.com/jancumps/stepper_driver_lib.git"
  GIT_TAG "origin/main"
)
FetchContent_MakeAvailable(stepper_driver)

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
        ${stepper_driver_SOURCE_DIR}/source/stepper_driver.cpp
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

pico_add_extra_outputs(${CMAKE_PROJECT_NAME} )