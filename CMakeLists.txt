cmake_minimum_required(VERSION 3.22)
include(FetchContent)

# Enable verbose compilation
set(CMAKE_VERBOSE_MAKEFILE ON)

# Ensure standard versions are set
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Enable automatic SDK download
set(PICO_SDK_FETCH_FROM_GIT on)

# Specify the Raspberry Pi Pico SDK
FetchContent_Declare(
    pico_sdk
    GIT_REPOSITORY https://github.com/raspberrypi/pico-sdk.git
    GIT_TAG master  # You can also specify a stable release tag
)

message("Making pico SDK availiable")
# Fetch and make it available
FetchContent_MakeAvailable(pico_sdk)

message("Including Pico sdk import...")
message("Pico SDK course Dir: ")
message(${pico_sdk_SOURCE_DIR})
# Import the Raspberry Pi Pico SDK
include(${pico_sdk_SOURCE_DIR}/external/pico_sdk_import.cmake)

message("Welcome to project BELUGA!")
project(BELUGA C CXX ASM)
# Initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# Fetch the Arduino-Pico Core
FetchContent_Declare(
    arduino_pico
    GIT_REPOSITORY https://github.com/earlephilhower/arduino-pico.git
    GIT_TAG master
)
FetchContent_MakeAvailable(arduino_pico)

# Fetch external libraries
include(ExternalProject)

# WiringPi
ExternalProject_Add(
  WiringPi
  GIT_REPOSITORY https://github.com/WiringPi/WiringPi.git
  GIT_TAG 3.10
  PREFIX ${CMAKE_BINARY_DIR}/wiringpi
  CONFIGURE_COMMAND ""
  BUILD_COMMAND cd <SOURCE_DIR> && ./build
  INSTALL_COMMAND ""
  BUILD_IN_SOURCE TRUE
)

# Adafruit SHT4X
ExternalProject_Add(
    adafruit_sht40
    GIT_REPOSITORY https://github.com/adafruit/Adafruit_SHT4X.git
    GIT_TAG master
    PREFIX ${CMAKE_BINARY_DIR}/sht40
    CONFIGURE_COMMAND ""  # No configuration step needed
    BUILD_COMMAND ""      # No build step here
    INSTALL_COMMAND ""    # No install step needed
    BUILD_IN_SOURCE TRUE  # Build in source directory
)

set(SHT40_SRC_DIR ${CMAKE_BINARY_DIR}/libsht40)

# Create a custom target to compile the SHT40 source files
add_custom_target(compile_adafruit_sht40 ALL
    COMMAND ${CMAKE_COMMAND} -E make_directory ${SHT40_SRC_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy_directory 
        ${CMAKE_BINARY_DIR}/sht40/src/adafruit_sht40 
        ${SHT40_SRC_DIR}
    COMMAND ${CMAKE_CXX_COMPILER} -c -I${SHT40_SRC_DIR} ${SHT40_SRC_DIR}/*.cpp -o ${SHT40_SRC_DIR}/libsht40.o
    DEPENDS adafruit_sht40
)

# Create a static library for SHT40
add_library(adafruit_sht40_lib STATIC ${SHT40_SRC_DIR}/libsht40.o)

# Link the compiled object files with Beluga
target_include_directories(adafruit_sht40_lib PUBLIC ${SHT40_SRC_DIR})

# Gather all C++ source files
file(GLOB SRC_FILES src/*.cpp)

# Define main executable
add_executable(beluga ${SRC_FILES})

# Include directories
target_include_directories(beluga PRIVATE
    ${arduino_pico_SOURCE_DIR}/cores/rp2040
    ${arduino_pico_SOURCE_DIR}/libraries/Wire
    ${CMAKE_BINARY_DIR}/wiringpi/src/WiringPi
    ${CMAKE_BINARY_DIR}/sht40/src/sht40
)

# Link libraries
target_link_libraries(beluga PRIVATE
    ${CMAKE_BINARY_DIR}/wiringpi/src/WiringPi/wiringPi
    adafruit_sht40_lib
    pico_stdlib
    hardware_i2c
)

# Ensure beluga depends on WiringPi and SHT40 so they build first
add_dependencies(beluga WiringPi adafruit_sht40)

# Create Binary and UF2 File
pico_add_extra_outputs(beluga)
