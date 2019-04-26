cmake_minimum_required(VERSION 2.8.3)
project(mdr_demo_simple_pick_and_place)

find_package(catkin REQUIRED COMPONENTS
  rospy
  mdr_move_arm_action
  mdr_move_base_action
  mdr_perceive_plane_action
  mdr_pickup_action
  mdr_place_action
  mas_execution_manager
  mdr_navigation_behaviours
  mdr_perception_behaviours
  mdr_manipulation_behaviours
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    mdr_move_arm_action
    mdr_move_base_action
    mdr_perceive_plane_action
    mdr_pickup_action
    mdr_place_action
    mas_execution_manager
    mdr_navigation_behaviours
    mdr_perception_behaviours
    mdr_manipulation_behaviours
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
