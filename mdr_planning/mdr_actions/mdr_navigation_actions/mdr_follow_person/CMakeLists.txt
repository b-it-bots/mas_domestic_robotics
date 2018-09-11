cmake_minimum_required(VERSION 2.8.3)
project(mdr_follow_person)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    geometry_msgs
    nav_msgs
    mcr_algorithms
    mcr_perception_msgs
    tf
    std_srvs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    mcr_algorithms
    mcr_perception_msgs
    std_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(follow_person_node
  ros/src/follow_person_node.cpp
)
add_dependencies(follow_person_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(follow_person_node
  ${catkin_LIBRARIES}
)


### TEST
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(ros/launch)
endif()


### INSTALLS
install(TARGETS follow_person_node follow_person_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
