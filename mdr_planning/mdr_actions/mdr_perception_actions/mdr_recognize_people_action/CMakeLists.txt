cmake_minimum_required(VERSION 2.8.3)
project(mdr_recognize_people_action)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslint
  actionlib
  actionlib_msgs
  mas_perception_msgs
)

catkin_python_setup()

add_action_files(DIRECTORY ros/action
  FILES
    RecognizePeople.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    mas_perception_msgs
)

catkin_package(
 CATKIN_DEPENDS
    actionlib_msgs
    mas_perception_msgs
)

###################
#  INSTALL
###################
install(PROGRAMS
  ros/scripts/recognize_people_action
  ros/scripts/recognize_people_client_test
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)