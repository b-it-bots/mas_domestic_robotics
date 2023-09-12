cmake_minimum_required(VERSION 2.8.3)
project(mdr_composite_behaviours)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslint
  smach
  mas_execution_manager
  mdr_rosplan_interface
  mas_hsr_head_controller
)

roslint_python()
catkin_python_setup()

catkin_package(
 CATKIN_DEPENDS
   rospy
   smach
   mas_execution_manager
   mdr_rosplan_interface
   mas_hsr_head_controller
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
