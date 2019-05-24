cmake_minimum_required(VERSION 2.8.3)
project(mdr_knowledge_behaviours)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslint
  smach
  mas_execution_manager
)

roslint_python()
catkin_python_setup()

catkin_package(
 CATKIN_DEPENDS
   rospy
   smach
   mas_execution_manager
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
