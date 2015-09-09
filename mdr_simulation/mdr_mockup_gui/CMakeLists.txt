cmake_minimum_required(VERSION 2.8.3)
project(mdr_mockup_gui)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    sensor_msgs
    mcr_speech_msgs
    std_msgs
)

catkin_python_setup()

catkin_package(
 CATKIN_DEPENDS
   sensor_msgs
   mcr_speech_msgs
   std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  ros/scripts/gui
  ros/scripts/speech_wrapper
  ros/src/${PROJECT_NAME}/gui.py
  ros/src/${PROJECT_NAME}/speech_wrapper.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
