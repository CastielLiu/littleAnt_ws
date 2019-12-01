cmake_minimum_required(VERSION 2.8.3)
project(array_msgs)

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
   UInt32Array.msg
   Float32Array.msg
 )




## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
		sensor_msgs
		std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES array_msgs
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

