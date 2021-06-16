cmake_minimum_required(VERSION 3.0.2)
project(driverless_actions)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  actionlib_msgs
  sensor_msgs
)

add_action_files(
  FILES
  DoDriverlessTask.action
  DoReverse.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

