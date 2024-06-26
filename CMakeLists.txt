cmake_minimum_required(VERSION 2.8.0)
project(Multi_Sensor_Fusion)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3")

option(TEST_DEBUG "option for debug output" OFF)
if (TEST_DEBUG)
    add_definitions(-DTEST_DEBUG)
    set(CMAKE_BUILD_TYPE Debug)
endif ()

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if (COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++11.")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif ()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

add_subdirectory(third_party/GeographicLib)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ${YAML_CPP_INCLUDE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/third_party/GeographicLib/include
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
        src/msf_core.cpp
        src/multiSensorsFusion.cpp
        src/imuGPSFusion.cpp
        src/baseKF.cpp
        src/msf_imu_processor.cpp
        src/msf_odom_processor.cpp
        src/msf_pose_processor.cpp
        src/msf_initializer.cpp
        src/msf_pos_processor.cpp
        src/msf_gps_processor.cpp)

target_link_libraries(${PROJECT_NAME} libGeographiccc yaml-cpp)