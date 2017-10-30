cmake_minimum_required(VERSION 2.8.3)
project(mdr_pick_and_place)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
)

install(PROGRAMS
  ros/scripts/pick_and_place_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/ros/scripts
)
