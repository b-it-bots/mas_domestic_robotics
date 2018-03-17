cmake_minimum_required(VERSION 2.8.3)
project(mdr_scenario_loader)

find_package(catkin REQUIRED COMPONENTS
  rospy
  smach
)

catkin_python_setup()
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
