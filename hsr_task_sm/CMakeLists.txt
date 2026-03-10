cmake_minimum_required(VERSION 3.0.2)
project(hsr_task_sm)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  actionlib
  mdr_perceive_plane_action
  mdr_pickup_action
  mdr_place_action
  mdr_move_base_action
  mdr_move_arm_action
  mas_perception_msgs
)

catkin_python_setup()

catkin_package()

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(PROGRAMS
  ros/scripts/pick_and_place_sm
  ros/scripts/perceive_only_sm
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
