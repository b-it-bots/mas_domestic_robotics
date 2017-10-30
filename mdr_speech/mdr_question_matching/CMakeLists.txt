cmake_minimum_required(VERSION 2.8.3)
project(mdr_question_matching)

find_package(catkin REQUIRED
  COMPONENTS
    roslint
    rospy
)



catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    std_msgs
)

roslint_python()

### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch)
endif()


### INSTALLS
install(PROGRAMS
  ros/scripts/question_matcher
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
