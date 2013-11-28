cmake_minimum_required(VERSION 2.8.3)
project(mdr_simple_grasp_planner)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
)

catkin_python_setup()

catkin_package(
)

install(PROGRAMS
  ros/scripts/grasp_planner_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_add_nosetests(ros/test)