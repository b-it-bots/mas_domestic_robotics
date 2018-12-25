cmake_minimum_required(VERSION 2.8.3)
project(mdr_navigation_behaviours)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslint
  smach
  mas_execution_manager
  mdr_move_arm_action
  mdr_move_base_action
)

roslint_python()
catkin_python_setup()

catkin_package(
 CATKIN_DEPENDS
   rospy
   smach
   mas_execution_manager
   mdr_move_arm_action
   mdr_move_base_action
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
