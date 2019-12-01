cmake_minimum_required(VERSION 2.8.3)
project(avoiding)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  little_ant_msgs
  roscpp
  rospy
  std_msgs
  gps_msgs
  array_msgs
  serial
  jsk_recognition_msgs
  ant_math
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES avoiding
#  CATKIN_DEPENDS little_ant_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(avoiding_node src/avoiding.cpp)
add_dependencies(avoiding_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(avoiding_node   ${catkin_LIBRARIES})

