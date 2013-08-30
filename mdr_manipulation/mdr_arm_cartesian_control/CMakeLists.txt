cmake_minimum_required(VERSION 2.8.3)
project(mdr_arm_cartesian_control)

find_package(catkin REQUIRED COMPONENTS mcr_arm_cartesian_control)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
