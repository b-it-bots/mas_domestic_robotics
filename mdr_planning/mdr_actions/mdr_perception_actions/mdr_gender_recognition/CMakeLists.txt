cmake_minimum_required(VERSION 2.8.3)
project(mdr_gender_recognition)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  genmsg
  message_generation
  roslint
  rospy
  sensor_msgs
  mdr_perception_msgs
  mdr_detect_person
)

catkin_python_setup()

add_action_files(DIRECTORY ros/action
  FILES
  GenderRecognition.action
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  mdr_perception_msgs
  actionlib_msgs
)

catkin_package(
 CATKIN_DEPENDS
   actionlib_msgs
   message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  ros/scripts/gender_recognition_action
  ros/scripts/gender_recognition_client_test
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)