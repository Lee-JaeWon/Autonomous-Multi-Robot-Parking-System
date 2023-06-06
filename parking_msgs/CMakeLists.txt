cmake_minimum_required(VERSION 3.0.2)
project(parking_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs nav_msgs actionlib std_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  action.msg
  miniSequence.msg
  robotProcess.msg
  Sequence.msg
  parkingDone.msg
  )

add_service_files(
  DIRECTORY srv
  FILES
  order.srv
  carNum.srv
)

add_action_files(
  FILES
  parkingOrder.action
  Planner2Tracker.action
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    nav_msgs
    actionlib
    )

catkin_package(
    CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
    nav_msgs
    actionlib
    )
