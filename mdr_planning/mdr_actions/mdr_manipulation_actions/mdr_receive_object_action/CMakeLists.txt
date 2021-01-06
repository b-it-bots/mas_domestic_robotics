cmake_minimum_required(VERSION 2.8.3)
project(mdr_receive_object_action)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslint
  actionlib
  actionlib_msgs
  genmsg
  message_generation
  geometry_msgs
  mas_execution_manager
)

roslint_python()
catkin_python_setup()

add_action_files(DIRECTORY ros/action
  FILES
  ReceiveObject.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
)

catkin_package(
 CATKIN_DEPENDS
   rospy
   actionlib
   actionlib_msgs
   message_runtime
   geometry_msgs
   mas_execution_manager
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  ros/scripts/receive_object_action
  ros/scripts/receive_object_action_client_test
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)
