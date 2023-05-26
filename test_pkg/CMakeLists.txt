cmake_minimum_required(VERSION 2.8.3) 
project(test_pkg) 
## 캐킨빌드를할때요구되는구성요소패키지이다.
## 의존성패키지로message_generation, std_msgs, roscpp이며이패키지들이존재하지않으면빌드도중에에러가난다.
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
add_message_files(FILES MsgTutorial.msg CarData.msg)
## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES  geometry_msgs   sensor_msgs   std_msgs
)
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES QS_Bot_Imu
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs message_runtime tf2 tf2_ros
#  DEPENDS system_lib
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
## 인클루드디렉터리를설정한다.

## topic_publisher노드에대한빌드옵션이다. 
## 실행파일, 타깃링크라이브러리, 추가의존성등을설정한다.


add_executable(path_publisher src/path_publisher.cpp)
add_dependencies(path_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_publisher ${catkin_LIBRARIES})


## add_executable(imu_pub src/imu_pub.cpp)
## add_dependencies(imu_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## target_link_libraries(imu_pub ${catkin_LIBRARIES})

add_executable(local_goal_pub src/local_goal_pub.cpp)
add_dependencies(local_goal_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(local_goal_pub ${catkin_LIBRARIES})
