cmake_minimum_required(VERSION 2.8.3)
project(esr_radar)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  esr_radar_msgs
  jsk_recognition_msgs
  roscpp
  rospy
  std_msgs
  can2serial
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp  
)

#include_directories(${can2serial_INCLUDE_DIRS})

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

	
add_executable(esr_radar src/ESR_radar.cpp )
add_dependencies(esr_radar  ${catkin_EXPORTED_TARGETS})
target_link_libraries(esr_radar ${catkin_LIBRARIES})

 
