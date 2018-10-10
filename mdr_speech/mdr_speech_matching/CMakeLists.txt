cmake_minimum_required(VERSION 2.8.3)
project(mdr_speech_matching)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

catkin_package(

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
