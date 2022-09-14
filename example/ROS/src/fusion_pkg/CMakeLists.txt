cmake_minimum_required(VERSION 3.0.2)
project(fusion_pkg)

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
)

find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vio_localization_fusion
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
include
${PROJECT_SOURCE_DIR}/../../../../include
${PROJECT_SOURCE_DIR}/../../../../third_party/GeographicLib/include
${catkin_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
)

set(LIBS
${EIGEN3_LIBS}
${PROJECT_SOURCE_DIR}/../../../../lib/libMulti_Sensor_Fusion.so
)

add_executable(mocap_t265_orb_fusion_node
src/mocap_t265_orb_fusion.cpp)

target_link_libraries(mocap_t265_orb_fusion_node
  ${catkin_LIBRARIES}
  ${LIBS}
)

add_executable(imu_gps_fusion_node
src/imu_gps_fusion.cpp)

target_link_libraries(imu_gps_fusion_node
  ${catkin_LIBRARIES}
  ${LIBS}
)

add_executable(orb_fusion_node
src/orb_fusion.cpp)

target_link_libraries(orb_fusion_node
  ${catkin_LIBRARIES}
  ${LIBS}
)

add_executable(mocap_fusion_node
src/mocap_fusion.cpp)

target_link_libraries(mocap_fusion_node
  ${catkin_LIBRARIES}
  ${LIBS}
)