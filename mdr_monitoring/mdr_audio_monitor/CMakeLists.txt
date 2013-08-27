cmake_minimum_required(VERSION 2.8.3)
project(mdr_audio_monitor)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  mdr_monitoring_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy 
)

install(PROGRAMS
  ros/scripts/mdr_audio_monitor
  ros/scr/mdr_audio_monitor/mdr_audio_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
