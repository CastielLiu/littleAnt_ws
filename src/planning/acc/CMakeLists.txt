cmake_minimum_required(VERSION 2.8.3)
project(acc)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  jsk_recognition_msgs
  little_ant_msgs
  esr_radar_msgs
  roscpp
  rospy
  std_msgs
  ant_math
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES acc
#  CATKIN_DEPENDS jsk_recognition_msgs little_ant_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(acc_node src/acc_esr.cpp)
add_dependencies(acc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(acc_node   ${catkin_LIBRARIES})
