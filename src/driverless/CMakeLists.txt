cmake_minimum_required(VERSION 2.8.3)
project(driverless)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ant_math
  diagnostic_msgs
  gps_msgs
  jsk_recognition_msgs
  little_ant_msgs
  esr_radar
  nav_msgs
  geometry_msgs
  message_filters
  message_generation
  euclidean_cluster
  roscpp
  rospy
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   TrackingState.msg
 )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   #   diagnostic_msgs#   gps_msgs#   jsk_recognition_msgs#   little_ant_msgs#   
   std_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES driverless
#  CATKIN_DEPENDS ant_math diagnostic_msgs gps_msgs jsk_recognition_msgs little_ant_msgs message_filters message_generation roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/driverless.cpp src/path_tracking.cpp src/car_following.cpp src/extern_control.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} /usr/local/lib/libtinyxml2.a)
