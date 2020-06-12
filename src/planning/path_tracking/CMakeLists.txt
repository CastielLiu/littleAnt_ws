cmake_minimum_required(VERSION 2.8.3)
project(path_tracking)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -I /usr/include/eigen3/)


find_package(catkin REQUIRED COMPONENTS
  little_ant_msgs
  roscpp
  rospy
  std_msgs
  gps_msgs
  serial
  jsk_recognition_msgs
  message_filters
  ant_math
  message_generation
  diagnostic_msgs
)

add_message_files(
	FILES
	State.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES path_tracking
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

add_executable(path_tracking_node src/path_tracking.cpp)
add_dependencies(path_tracking_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_tracking_node   ${catkin_LIBRARIES})
