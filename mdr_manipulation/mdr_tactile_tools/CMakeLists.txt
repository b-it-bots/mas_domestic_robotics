cmake_minimum_required(VERSION 2.8.3)
project(mdr_tactile_tools)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    rosunit
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    schunk_sdh
    cob_srvs
    rosunit
)

install(PROGRAMS
  ros/scripts/grasp_evaluators_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(ros/test/grasp_evaluators_test.py)
endif()
