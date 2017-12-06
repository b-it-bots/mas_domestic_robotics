cmake_minimum_required(VERSION 2.8.3)
project(mdr_detect_person)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  genmsg
  std_msgs
  message_generation
  roslint
  rospy
  cob_script_server
  sensor_msgs
)

catkin_python_setup()

add_action_files(
  DIRECTORY
  ros/action
  FILES
  DetectPerson.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  actionlib_msgs
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

roslint_python()  # pep8 linting
roslint_cpp()     # ROS wrapper of Google's cpplint

install(PROGRAMS
  ros/scripts/detect_person_action
  ros/scripts/detect_person_action_client_test
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)

