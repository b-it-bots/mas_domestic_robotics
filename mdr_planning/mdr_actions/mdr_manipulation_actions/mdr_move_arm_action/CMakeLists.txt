cmake_minimum_required(VERSION 2.8.3)
project(mdr_move_arm_action)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslint
  actionlib
  actionlib_msgs
  genmsg
  message_generation
  smach
  smach_ros
  geometry_msgs
  moveit_commander
)

roslint_python()
catkin_python_setup()

add_action_files(DIRECTORY ros/action
  FILES
  MoveArm.action
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
   smach
   smach_ros
   geometry_msgs
   moveit_commander
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  ros/scripts/move_arm_action
  ros/scripts/move_arm_action_client_test
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)
