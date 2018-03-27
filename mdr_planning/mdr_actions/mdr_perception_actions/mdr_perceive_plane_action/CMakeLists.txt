cmake_minimum_required(VERSION 2.8.3)
project(mdr_perceive_plane_action)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  genmsg
  message_generation
  roslint
  rospy
  roslaunch
  mcr_perception_msgs
)

catkin_python_setup()

add_action_files(DIRECTORY ros/action
  FILES
    PerceivePlane.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  mcr_perception_msgs
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
    ros/scripts/perceive_plane_action
    ros/scripts/perceive_plane_client_test
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)

roslint_python()
roslaunch_add_file_check(ros/launch)

