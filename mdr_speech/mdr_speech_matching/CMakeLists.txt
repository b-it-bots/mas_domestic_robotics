cmake_minimum_required(VERSION 2.8.3)
project(mdr_speech_matching)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    std_msgs
    message_generation
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
    MatchedSentence.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS message_runtime
)

#############
## Install ##
#############

# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mdr_speech_to_text_matching.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
