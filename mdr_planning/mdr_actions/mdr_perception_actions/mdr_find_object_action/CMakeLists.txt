cmake_minimum_required(VERSION 2.8.3)
project(mdr_find_object_action)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslint
  genmsg
  message_generation
  actionlib
  actionlib_msgs
  mas_perception_libs
  mas_perception_msgs
  mas_knowledge_base
  mdr_move_base_action
  mdr_perceive_plane_action
)

catkin_python_setup()
roslint_python()

add_action_files(DIRECTORY ros/action
  FILES
    FindObject.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    mas_perception_msgs
)

catkin_package(
 CATKIN_DEPENDS
   rospy
   actionlib
   actionlib_msgs
   message_runtime
   mas_perception_libs
   mas_perception_msgs
   mas_knowledge_base
   mdr_move_base_action
   mdr_perceive_plane_action
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
    ros/scripts/find_object_action
    ros/scripts/find_object_action_client_test
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
