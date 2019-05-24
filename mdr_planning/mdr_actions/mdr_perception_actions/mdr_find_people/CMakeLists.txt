cmake_minimum_required(VERSION 2.8.3)
project(mdr_find_people)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslint
  roslaunch
  actionlib
  actionlib_msgs
  message_generation
  mas_perception_msgs
  mas_perception_libs
)

catkin_python_setup()
roslint_python()
roslaunch_add_file_check(ros/launch)

add_action_files(DIRECTORY ros/action
  FILES
  FindPeople.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    mas_perception_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
 CATKIN_DEPENDS
   actionlib_msgs
   message_runtime
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mdr_question_answering.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

