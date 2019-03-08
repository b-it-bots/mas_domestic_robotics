cmake_minimum_required(VERSION 2.8.3)
project(mdr_monitoring_msgs)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  message_generation
  std_msgs
)

add_message_files(
  FILES
  AudioEndpoint.msg
  Endpoints.msg
  ExecutionState.msg
  Event.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  rospy
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
