cmake_minimum_required(VERSION 2.8.3)
project(mdr_mockup_gui)

find_package(catkin REQUIRED COMPONENTS roscpp rospy sensor_msgs mcr_speech_msgs)

catkin_python_setup()

generate_messages(
   DEPENDENCIES
   sensor_msgs
   mcr_speech_msgs
)
catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  ros/scripts/mdr_mockup_gui
  ros/src/mdr_mockup_gui/mdr_mockup_gui.py
  ros/scripts/mdr_mockup_speech_wrapper
  ros/src/mdr_mockup_gui/mdr_mockup_speech_wrapper.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
