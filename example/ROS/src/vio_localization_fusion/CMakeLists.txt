cmake_minimum_required(VERSION 3.0.2)
project(vio_localization_fusion)

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

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vio_localization_fusion
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf
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

add_executable(t265_localization_fusion_node 
src/t265_localization_fusion_node.cpp
src/t265_localization_wrapper.cpp)

target_link_libraries(t265_localization_fusion_node
  ${catkin_LIBRARIES}
  ${LIBS}
)

add_executable(t265_fusion_node 
src/t265_fusion.cpp)

target_link_libraries(t265_fusion_node
  ${catkin_LIBRARIES}
  ${LIBS}
)

add_executable(t265_fusion_new_node 
src/t265_fusion_new.cpp)

target_link_libraries(t265_fusion_new_node
  ${catkin_LIBRARIES}
  ${LIBS}
)

add_executable(jkw_t265_fusion_node 
src/jkw_t265_fusion.cpp)

target_link_libraries(jkw_t265_fusion_node
  ${catkin_LIBRARIES}
  ${LIBS}
)

add_executable(t265_d400_fusion_node 
src/t265_d400_fusion.cpp)

target_link_libraries(t265_d400_fusion_node
  ${catkin_LIBRARIES}
  ${LIBS}
)

add_executable(jkw_drone_fusion_node 
src/jkw_drone_fusion.cpp)

target_link_libraries(jkw_drone_fusion_node
  ${catkin_LIBRARIES}
  ${LIBS}
)
