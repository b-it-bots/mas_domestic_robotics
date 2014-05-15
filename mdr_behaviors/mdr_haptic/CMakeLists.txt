cmake_minimum_required(VERSION 2.8.3)
project(mdr_haptic)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    mcr_algorithms
    nav_msgs
    rospy
    std_srvs
    tf
)

catkin_python_setup()

install(PROGRAMS
  ros/scripts/mdr_haptic_guidance_node
  ros/src/mdr_haptic/mdr_haptic_guidance.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
