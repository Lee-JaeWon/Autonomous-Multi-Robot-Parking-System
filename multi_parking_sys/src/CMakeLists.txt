cmake_minimum_required(VERSION 3.0.2)
project(odom_test)
## 캐킨빌드를할때요구되는구성요소패키지이다.
## 의존성패키지로message_generation, std_msgs, roscpp이며이패키지들이존재하지않으면빌드도중에에러가난다.
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  dynamixel_workbench_msgs
  dynamixel_workbench_toolbox
  cmake_modules
  nav_msgs
  tf2 
  serial
  tf2_ros
  dynamixel_sdk

)
## 메시지선언: MsgTutorial.msg 
find_package(Eigen3 REQUIRED)

## std_msgs가설치되어있지않다면빌드도중에에러가난다.
generate_messages(DEPENDENCIES std_msgs nav_msgs sensor_msgs)
## 캐킨패키지옵션으로라이브러리, 캐킨빌드의존성, 시스템의존패키지를기술한다. 
catkin_package( 
#include "dynamixel_workbench_msgs/DynamixelStateList.h"
LIBRARIES odom_test dynamixel_workbench_msgs dynamixel_workbench tf2 tf2_ros dynamixel_sdk sensor_msgs
 CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs dynamixel_workbench_msgs trajectory_msgs dynamixel_workbench_toolbox cmake_modules nav_msgs tf2 tf2_ros dynamixel_sdk
)

## 인클루드디렉터리를설정한다.
include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
## topic_publisher노드에대한빌드옵션이다. 
## 실행파일, 타깃링크라이브러리, 추가의존성등을설정한다.
add_executable(odom_node src/odometry.cpp)
add_dependencies(odom_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_node ${catkin_LIBRARIES})

add_executable(imu_pub src/imu_pub.cpp)
add_dependencies(imu_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(imu_pub ${catkin_LIBRARIES})