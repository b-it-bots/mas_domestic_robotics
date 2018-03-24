cmake_minimum_required(VERSION 2.8.3)
project(mdr_store_groceries)

find_package(catkin REQUIRED COMPONENTS
  roslint
  rospy
  smach_ros
  smach
  actionlib
  std_msgs
  mdr_pickup_action
  mdr_place_action
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    smach
    smach_ros
    actionlib
    std_msgs
    mdr_pickup_action
    mdr_place_action
)

catkin_python_setup()
roslint_python()

install(PROGRAMS
  ros/scripts/store_groceries
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/ros/scripts
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
