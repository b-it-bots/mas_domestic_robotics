cmake_minimum_required(VERSION 2.8.3)
project(mdr_demo_lab_manager)

find_package(catkin REQUIRED COMPONENTS
  rospy
  mdr_move_arm_action
  mdr_move_base_action
  mdr_detect_person
  mas_execution_manager
  mdr_navigation_behaviours
  mdr_perception_behaviours
  mdr_manipulation_behaviours
  mas_knowledge_base
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    mdr_move_arm_action
    mdr_move_base_action
    mdr_detect_person
    mas_execution_manager
    mdr_navigation_behaviours
    mdr_perception_behaviours
    mdr_manipulation_behaviours
    mas_knowledge_base
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
