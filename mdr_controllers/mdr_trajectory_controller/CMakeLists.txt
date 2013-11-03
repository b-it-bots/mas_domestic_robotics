cmake_minimum_required(VERSION 2.8.3)
project(mdr_trajectory_controller)

find_package(catkin REQUIRED
  COMPONENTS
    controller_interface
    hardware_interface
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    controller_interface
    hardware_interface
    std_msgs
)

include_directories(common/include)

## Declare a cpp library
# add_library(mdr_trajectory_controller
#   src/${PROJECT_NAME}/mdr_trajectory_controller.cpp
# )

## Declare a cpp executable
# add_executable(mdr_trajectory_controller_node src/mdr_trajectory_controller_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(mdr_trajectory_controller_node mdr_trajectory_controller_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(mdr_trajectory_controller_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS mdr_trajectory_controller mdr_trajectory_controller_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mdr_trajectory_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
