cmake_minimum_required(VERSION 2.8.3)
project(mdr_demo_patrol)

find_package(catkin REQUIRED COMPONENTS
  mdr_move_arm_action
  mdr_move_base_action
  mdr_navigation_behaviours
)

catkin_package(
  CATKIN_DEPENDS
    mdr_move_arm_action
    mdr_move_base_action
    mdr_navigation_behaviours
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
