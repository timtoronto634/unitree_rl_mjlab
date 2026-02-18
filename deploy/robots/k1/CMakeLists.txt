cmake_minimum_required(VERSION 3.12)
project(k1_controller)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    set(ONNXRUNTIME_DIR "${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-x64-1.22.0")
    set(BOOSTER_SDK_LIB_DIR "${PROJECT_SOURCE_DIR}/../../../booster_robotics_sdk/lib/x86_64")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    set(ONNXRUNTIME_DIR "${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-aarch64-1.22.0")
    set(BOOSTER_SDK_LIB_DIR "${PROJECT_SOURCE_DIR}/../../../booster_robotics_sdk/lib/aarch64")
endif()

set(BOOSTER_SDK_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/../../../booster_robotics_sdk/include")

include_directories(
  /usr/include/eigen3
  # Unitree SDK2 headers: needed for UnitreeJoystick, RecurrentThread,
  # GetCurrentTimeMillisecond used in shared deploy/include/ FSM headers.
  /usr/local/include/ddscxx
  /usr/local/include/iceoryx/v2.0.2
  include
  ${ONNXRUNTIME_DIR}/include
  ${PROJECT_SOURCE_DIR}/../../include/
  ${BOOSTER_SDK_INCLUDE_DIR}
)

link_libraries(
  # Unitree SDK2: required for shared FSM utilities (RecurrentThread,
  # GetCurrentTimeMillisecond, UnitreeJoystick). NOT used for DDS comms.
  unitree_sdk2 ddsc ddscxx
  # Booster SDK: used for actual DDS communication with K1 hardware.
  ${BOOSTER_SDK_LIB_DIR}/libbooster_robotics_sdk.a
  fastrtps fastcdr
  rt pthread
  Boost::program_options
  yaml-cpp
  fmt
  ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so.1.22.0
)

file(GLOB_RECURSE ADD_SRC_LIST
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)

add_library(${PROJECT_NAME}_lib ${ADD_SRC_LIST})
link_libraries(${PROJECT_NAME}_lib)

add_executable(k1_ctrl main.cpp)
