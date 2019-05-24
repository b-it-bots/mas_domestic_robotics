cmake_minimum_required(VERSION 2.8.3)
project(mdr_robot_inspection)

find_package(catkin REQUIRED COMPONENTS
  roslint
  rospy
  smach_ros
  smach
  actionlib
  std_msgs
  mdr_door_status
  mdr_enter_door_action
  mdr_move_base_action
  mas_execution_manager
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    smach
    smach_ros
    actionlib
    std_msgs
    zbar_ros
    mdr_door_status
    mdr_move_base_action
    mdr_enter_door_action
    mas_execution_manager
)

catkin_python_setup()
roslint_python()

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
