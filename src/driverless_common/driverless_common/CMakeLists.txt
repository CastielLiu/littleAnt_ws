cmake_minimum_required(VERSION 3.0.2)
project(driverless_common)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  actionlib_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   VehicleState.msg
   VehicleCtrlCmd.msg
   SystemState.msg
 )
 
 add_action_files(
  FILES
  DoDriverlessTask.action
  DoReverse.action
)
 
 
## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	std_msgs
	actionlib_msgs
	geometry_msgs
)
 
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES driverless_common
  CATKIN_DEPENDS roscpp rospy message_runtime
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   include/${PROJECT_NAME}/structs.h
# )


 add_executable(${PROJECT_NAME}_example src/example.cpp)

 add_dependencies(${PROJECT_NAME}_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_example
   ${catkin_LIBRARIES}
 )

