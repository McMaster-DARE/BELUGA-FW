cmake_minimum_required(VERSION 3.22)

set(CMAKE_VERBOSE_MAKEFILE ON)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)

# Print the C++ standard being used
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")

set(DEP_PATH ${CMAKE_SOURCE_DIR}/dep)

set(PICO_SDK_PATH ${DEP_PATH}/pico-sdk)
# set(WIRINGPI_PATH ${DEP_PATH}/WiringPi-3.10)

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(BELUGA C CXX ASM)

pico_sdk_init()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

file(GLOB SRC_FILES src/*.cpp)

# Define the executable
add_executable(beluga ${SRC_FILES})

# Include WiringPi headers
target_include_directories(beluga PRIVATE 
    # ${WIRINGPI_PATH}/wiringPi
    ${PICO_SDK_PATH}/src/common/pico_base/include
    ${PICO_SDK_PATH}/src/rp2_common/hardware_gpio/include
    ${PICO_SDK_PATH}/src/rp2_common/hardware_pwm/include
    )

# WiringPi Build
# add_custom_target(WiringPi_Build
#     COMMAND cd ${WIRINGPI_PATH} && ./build
#     WORKING_DIRECTORY ${WIRINGPI_PATH}
# )

# /home/marali/BELUGA-FW/dep/pico-sdk/src/rp2_common/hardware_gpio/include/hardware
# /home/marali/BELUGA-FW/dep/pico-sdk/src/rp2_common/hardware_pwm/include/hardware
#     ${PICO_SDK_PATH}/src/common/pico_base/include
#     ${PICO_SDK_PATH}/src/rp2_common/hardware_gpio/include
#     ${PICO_SDK_PATH}/src/rp2_common/hardware_pwm/include

# Add WiringPi as a shared library
# add_library(wiringPi_lib SHARED IMPORTED)
# set_target_properties(wiringPi_lib PROPERTIES
#     IMPORTED_LOCATION ${WIRINGPI_PATH}/wiringPi/libwiringPi.so.3.10
# )

# # Ensure WiringPi builds first
# add_dependencies(wiringPi_lib WiringPi_Build)

# Link libraries
target_link_libraries(beluga PRIVATE
    # wiringPi_lib
    pico_stdlib
    hardware_pwm
    hardware_gpio
    hardware_i2c
)

# Create Binary and UF2 File
pico_add_extra_outputs(beluga)