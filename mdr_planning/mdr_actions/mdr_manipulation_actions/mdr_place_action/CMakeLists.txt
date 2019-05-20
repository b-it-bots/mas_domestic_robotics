cmake_minimum_required(VERSION 2.8.3)
project(mdr_place_action)

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
  trajectory_msgs
  rosplan_dispatch_msgs
  rosplan_knowledge_msgs
  diagnostic_msgs
  mas_perception_msgs
  mdr_rosplan_interface
  mdr_move_arm_action
  mdr_move_base_action
)

roslint_python()
catkin_python_setup()

add_action_files(DIRECTORY ros/action
  FILES
  Place.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
)

catkin_package(
 CATKIN_DEPENDS
   rospy
   roslint
   actionlib
   actionlib_msgs
   smach
   smach_ros
   tf
   geometry_msgs
   trajectory_msgs
   diagnostic_msgs
   mas_perception_msgs
   mdr_rosplan_interface
   mdr_move_arm_action
   mdr_move_base_action
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  ros/scripts/place_action
  ros/scripts/place_action_client_test
  ros/scripts/place_client
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)
