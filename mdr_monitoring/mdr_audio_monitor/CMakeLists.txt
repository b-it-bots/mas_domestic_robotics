cmake_minimum_required(VERSION 2.8.3)
project(mdr_audio_monitor)

find_package(catkin REQUIRED COMPONENTS rospy mdr_monitoring_msgs)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy mdr_monitoring_msgs
)

install(PROGRAMS
  ros/scripts/mdr_audio_node
  ros/src/mdr_audio_monitor/mdr_audio_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
