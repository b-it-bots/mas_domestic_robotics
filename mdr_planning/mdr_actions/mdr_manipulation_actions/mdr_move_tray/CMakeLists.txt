cmake_minimum_required(VERSION 2.8.3)
project(mdr_move_tray)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  genmsg
  message_generation
  roslint
  rospy
  cob_script_server
)

catkin_python_setup()

add_action_files(
  FILES
  MoveTray.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
)

catkin_package(
 CATKIN_DEPENDS
   actionlib_msgs
   message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
