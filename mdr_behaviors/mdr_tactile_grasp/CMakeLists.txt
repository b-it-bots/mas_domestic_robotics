cmake_minimum_required(VERSION 2.8.3)
project(mdr_tactile_grasp)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy  
    control_msgs
    schunk_sdh    
)

install(PROGRAMS
  ros/scripts/monitors_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
