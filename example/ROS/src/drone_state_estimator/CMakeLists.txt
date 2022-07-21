cmake_minimum_required(VERSION 3.0.2)
project(drone_state_estimator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  message_generation
  eigen_conversions
)

find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES drone_state_estimator
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs tf
#  DEPENDS system_lib
)

include_directories(
  include
  "/home/antusheng/research/map_based_localization/test/Multi_Sensor_Fusion/include"
  "/home/antusheng/research/map_based_localization/test/Multi_Sensor_Fusion/third_party/GeographicLib/include"
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

set(LIBS
${EIGEN3_LIBS}
/home/antusheng/research/map_based_localization/test/Multi_Sensor_Fusion/lib/libMulti_Sensor_Fusion.so
)

add_executable(state_estimator_node 
src/state_estimator_node.cpp
src/state_estimator_warpper.cpp)

target_link_libraries(state_estimator_node
  ${catkin_LIBRARIES}
  ${LIBS}
)
