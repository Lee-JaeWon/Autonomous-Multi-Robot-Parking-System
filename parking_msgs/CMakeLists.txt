cmake_minimum_required(VERSION 3.0.2)
project(parking_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  action.msg
  miniSequence.msg
  robotProcess.msg
  Sequence.msg
  )

add_service_files(
  DIRECTORY srv
  FILES
  order.srv
  carNum.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    )

catkin_package(
    CATKIN_DEPENDS
    message_runtime
    std_msgs
    )
