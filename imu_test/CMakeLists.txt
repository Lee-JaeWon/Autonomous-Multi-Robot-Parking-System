cmake_minimum_required(VERSION 3.0.2)
project(imu_test)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  serial
  tf2
  tf2_ros
)

## Generate messages in the 'msg' folder
# add_message_files(FILES imu.msg)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES  geometry_msgs   sensor_msgs   std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES QS_Bot_Imu
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs message_runtime tf2 tf2_ros
#  DEPENDS system_lib
)
include_directories(include  ${catkin_INCLUDE_DIRS})

add_executable(imu_pub src/imu_pub.cpp)

target_link_libraries(imu_pub ${catkin_LIBRARIES})
