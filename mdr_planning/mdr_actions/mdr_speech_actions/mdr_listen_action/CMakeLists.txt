cmake_minimum_required(VERSION 2.8.3)
project(mdr_listen_action)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  genmsg
  message_generation
  roslint
  rospy
  mdr_speech_matching
  mdr_speech_recognition
)

catkin_python_setup()
roslint_python()

add_action_files(DIRECTORY ros/action
  FILES
  Listen.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
)

catkin_package(
 CATKIN_DEPENDS
   actionlib_msgs
   message_runtime
   std_msgs
   mdr_speech_matching
   mdr_speech_recognition
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  ros/scripts/listen
  ros/scripts/listen_client_test
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)
