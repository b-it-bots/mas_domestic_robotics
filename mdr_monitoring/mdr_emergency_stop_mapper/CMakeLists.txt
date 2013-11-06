cmake_minimum_required(VERSION 2.8.3)
project(mdr_emergency_stop_mapper)

find_package(catkin REQUIRED
  COMPONENTS
    std_msgs
    cob_relayboard
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    cob_relayboard
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(emergency_stop_mapper
  ros/src/emergency_stop_mapper.cpp
)

target_link_libraries(emergency_stop_mapper
  ${catkin_LIBRARIES}
)

add_dependencies(emergency_stop_mapper
  ${catkin_EXPORTED_TARGETS}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS mdr_emergency_stop_translator mdr_emergency_stop_translator_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mdr_emergency_stop_translator.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
