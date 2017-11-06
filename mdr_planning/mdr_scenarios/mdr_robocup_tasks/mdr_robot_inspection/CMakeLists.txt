cmake_minimum_required(VERSION 2.8.3)
project(mdr_robot_inspection)

find_package(catkin REQUIRED COMPONENTS
  roslint
  rospy
  smach_ros
  smach
  actionlib
  std_msgs
  mdr_enter_door_action
  mdr_introduce_self_action
  mdr_move_base_safe
  mdr_move_tray_action
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    smach
    smach_ros
    actionlib
    std_msgs
    zbar_ros
    # vocon_speech_recognizer
    mcr_door_status
    mdr_bringup
    mdr_2dnav
    mdr_question_matching
    mdr_move_base_safe
    mdr_enter_door_action
    mdr_introduce_self_action
    mdr_move_tray_action
)

catkin_python_setup()

# ROS Lint
roslint_python()

### TESTS
#if(CATKIN_ENABLE_TESTING)
#  find_package(roslaunch REQUIRED)
#  roslaunch_add_file_check(ros/launch pc1:=localhost pc2:=localhost)
#endif()

### INSTALLS
install(PROGRAMS
  ros/scripts/robot_inspection
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/ros/scripts
)
install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
