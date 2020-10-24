cmake_minimum_required(VERSION 2.8.3)
project(mdr_demo_context_aware_hand_over)

find_package(catkin REQUIRED COMPONENTS
  rospy
  mdr_move_arm_action
  mdr_move_base_action
  mdr_find_people
  mdr_hand_over_action
  mas_execution_manager
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    mdr_move_arm_action
    mdr_move_base_action
    mdr_find_people
    mdr_hand_over_action
    mas_execution_manager
)

catkin_python_setup()

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
