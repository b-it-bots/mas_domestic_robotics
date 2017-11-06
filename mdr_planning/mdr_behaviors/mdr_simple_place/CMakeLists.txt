cmake_minimum_required(VERSION 2.8.3)
project(mdr_simple_place)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    mdr_behavior_msgs
    moveit_msgs
    sensor_msgs
)

install(PROGRAMS
  ros/scripts/simple_place_node
  ros/tools/simple_place_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
