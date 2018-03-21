cmake_minimum_required(VERSION 2.8.3)
project(mdr_execution_manager)

find_package(catkin REQUIRED COMPONENTS
  rospy
  smach
  roslint
)

roslint_python()
catkin_python_setup()
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
