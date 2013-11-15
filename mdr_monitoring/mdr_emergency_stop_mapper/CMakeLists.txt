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