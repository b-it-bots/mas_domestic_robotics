cmake_minimum_required(VERSION 2.8.3)
project(mdr_find_people)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslint
  roslaunch
  actionlib
  actionlib_msgs
  message_generation
  mas_perception_msgs
  mas_perception_libs
)

catkin_python_setup()
roslint_python()
roslaunch_add_file_check(ros/launch)

add_action_files(DIRECTORY ros/action
  FILES
  FindPeople.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    mas_perception_msgs
)

catkin_package(
 CATKIN_DEPENDS
   actionlib_msgs
   message_runtime
)

install(PROGRAMS
  ros/scripts/find_people_server
  ros/scripts/find_people_client_test
  ros/scripts/find_people_client
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
