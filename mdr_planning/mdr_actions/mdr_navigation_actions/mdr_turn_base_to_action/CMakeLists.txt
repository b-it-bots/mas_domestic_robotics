cmake_minimum_required(VERSION 2.8.3)
project(mdr_turn_base_to_action)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslint
  actionlib
  actionlib_msgs
  genmsg
  message_generation
  smach
  smach_ros
  tf
  geometry_msgs
  mdr_move_base_action
)

catkin_python_setup()
roslint_python()

add_action_files(DIRECTORY ros/action/
  FILES
  TurnBaseTo.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
)

catkin_package(
 CATKIN_DEPENDS
   rospy
   actionlib
   actionlib_msgs
   actionlib_msgs
   message_runtime
   smach
   smach_ros
   tf
   geometry_msgs
   mdr_move_base_action
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  ros/scripts/turn_base_to_action
  ros/scripts/turn_base_to_action_client_test
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)
