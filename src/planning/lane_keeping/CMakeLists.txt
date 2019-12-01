cmake_minimum_required(VERSION 2.8.3)
project(lane_keeping)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  little_ant_msgs
  roscpp
  rospy
  std_msgs
  ant_math
  gps_msgs
)


## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lane_keeping
#  CATKIN_DEPENDS little_ant_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
	include
	${catkin_INCLUDE_DIRS}
)



add_executable(lane_keeping_node src/lane_keeping.cpp)
add_dependencies(lane_keeping_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lane_keeping_node  ${catkin_LIBRARIES})


