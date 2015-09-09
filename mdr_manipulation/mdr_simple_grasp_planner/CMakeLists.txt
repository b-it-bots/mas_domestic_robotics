cmake_minimum_required(VERSION 2.8.3)
project(mdr_simple_grasp_planner)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
)

install(PROGRAMS
  ros/scripts/grasp_planner_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

### TEST
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

  roslaunch_add_file_check(ros/launch)
  add_rostest(ros/launch/grasp_planner.test)
endif()
