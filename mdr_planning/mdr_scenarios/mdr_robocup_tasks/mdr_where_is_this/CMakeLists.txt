cmake_minimum_required(VERSION 2.8.3)
project(mdr_where_is_this)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslint
  mas_execution_manager
  mdr_listen_action
  mdr_speech_matching
  topological_map_ros
  geometry_msgs
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    mas_execution_manager
    mdr_listen_action
    mdr_speech_matching
    topological_map_ros
    geometry_msgs
    std_msgs
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
