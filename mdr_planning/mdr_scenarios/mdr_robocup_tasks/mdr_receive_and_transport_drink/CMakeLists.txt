cmake_minimum_required(VERSION 2.8.3)
project(mdr_receive_and_transport_drink)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslint
  mas_execution_manager
  mdr_find_people
  mdr_move_base_action
  mdr_receive_object_action
  mdr_perceive_plane_action
  mdr_place_action
  mdr_navigation_behaviours
  mdr_perception_behaviours
  mdr_manipulation_behaviours
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    mas_execution_manager
    mdr_find_people
    mdr_move_base_action
    mdr_receive_object_action
    mdr_perceive_plane_action
    mdr_place_action
    mdr_navigation_behaviours
    mdr_perception_behaviours
    mdr_manipulation_behaviours
)

catkin_python_setup()
roslint_python()

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
