cmake_minimum_required(VERSION 2.8.3)
project(mdr_serve_drinks)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslint
  mas_execution_manager
  mdr_move_base_action
  mdr_move_arm_action
  mdr_pickup_action
  mdr_perceive_plane_action
  mdr_navigation_behaviours
  mdr_perception_behaviours
  mdr_manipulation_behaviours
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    mas_execution_manager
    mdr_move_base_action
    mdr_move_arm_action
    mdr_pickup_action
    mdr_perceive_plane_action
    mdr_navigation_behaviours
    mdr_perception_behaviours
    mdr_manipulation_behaviours
)

catkin_python_setup()
roslint_python()

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
