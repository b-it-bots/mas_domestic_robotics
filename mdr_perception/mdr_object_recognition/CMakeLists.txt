cmake_minimum_required(VERSION 2.8.3)
project(mdr_object_recognition)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED
  COMPONENTS
    mcr_object_recognition_mean_circle
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    roslint
    sensor_msgs
    tmc_tabletop_segmentator
)

roslint_python()

install(PROGRAMS
  ros/scripts/hsr_object_detector
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
