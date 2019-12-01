cmake_minimum_required(VERSION 2.8.3)
project(esr_radar_msgs)

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
  Object.msg
  Objects.msg
 )




## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
		sensor_msgs
		std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES little_ant_msgs
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

