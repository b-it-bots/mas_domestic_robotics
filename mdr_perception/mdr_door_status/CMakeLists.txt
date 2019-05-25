cmake_minimum_required(VERSION 2.8.3)
project(mdr_door_status)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs rosconsole)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs rosconsole
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(door_status_node ros/src/door_status_node.cpp)

target_link_libraries(door_status_node
   ${catkin_LIBRARIES}
)

add_executable(door_status_node_mockup ros/src/door_status_node_mockup.cpp)

target_link_libraries(door_status_node_mockup
   ${catkin_LIBRARIES}
)


### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch)
endif()


### INSTALLS
install(
  TARGETS
    door_status_node
    door_status_node_mockup
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
