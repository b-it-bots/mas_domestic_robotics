cmake_minimum_required(VERSION 2.8.3)
project(mdr_recognize_emotion_action)

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
  RecognizeEmotion.action
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
   mdr_perception_msgs
   sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  ros/scripts/recognize_emotion_action
  ros/scripts/recognize_emotion_action_client_test
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)
