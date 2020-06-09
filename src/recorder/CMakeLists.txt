cmake_minimum_required(VERSION 2.8.3)
project(recorder)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  little_ant_msgs
  roscpp
  rospy
  std_msgs
  gps_msgs
  serial
  jsk_recognition_msgs
  ant_math
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES record_gps_data
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

add_executable(record_path_node src/record_path.cpp)
add_dependencies(record_path_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(record_path_node   ${catkin_LIBRARIES})

add_executable(record_vehicle_state_node src/record_vehicle_state.cpp)
add_dependencies(record_vehicle_state_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(record_vehicle_state_node   ${catkin_LIBRARIES})

add_executable(record_IMU_node src/record_imu.cpp)
add_dependencies(record_IMU_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(record_IMU_node   ${catkin_LIBRARIES})

add_executable(record_drive_status_node src/record_drive_status.cpp)
add_dependencies(record_drive_status_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(record_drive_status_node   ${catkin_LIBRARIES})

