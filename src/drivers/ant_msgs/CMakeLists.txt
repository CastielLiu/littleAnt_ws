cmake_minimum_required(VERSION 2.8.3)
project(ant_msgs)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   ControlCmd1.msg
   ControlCmd2.msg
   State.msg
   State1.msg
   State2.msg
   State3.msg
   State4.msg
 )


## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
		sensor_msgs
		std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ant_msgs
  CATKIN_DEPENDS message_runtime roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

