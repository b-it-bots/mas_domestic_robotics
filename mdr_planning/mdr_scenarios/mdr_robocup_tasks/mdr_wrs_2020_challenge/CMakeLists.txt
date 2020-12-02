cmake_minimum_required(VERSION 2.8.3)
project(mdr_wrs_2020_challenge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslint
  std_msgs
  geometry_msgs
  topological_map_ros
  mas_execution_manager
  mas_knowledge_base
  mdr_listen_action
  mdr_speech_matching
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    topological_map_ros
    mas_execution_manager
    mas_knowledge_base
    mdr_listen_action
    mdr_speech_matching
)

include_directories()

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
