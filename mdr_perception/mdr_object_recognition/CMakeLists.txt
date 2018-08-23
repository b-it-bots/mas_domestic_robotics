cmake_minimum_required(VERSION 2.8.3)
project(mdr_object_recognition)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    std_msgs
    sensor_msgs
    mcr_object_recognition_mean_circle
    mas_perception_libs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
