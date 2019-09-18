cmake_minimum_required(VERSION 2.8.3)
project(mdr_sciroc_door_open)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslint
  std_msgs
  geometry_msgs
  mas_execution_manager
  mas_knowledge_base
  handle_detector
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    mas_execution_manager
    mas_knowledge_base
    handle_detector
    mdr_move_forward_action
    mdr_move_arm_action
)

include_directories()

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
