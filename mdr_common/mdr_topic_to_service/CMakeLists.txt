cmake_minimum_required(VERSION 2.8.3)
project(mdr_topic_to_service)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    mcr_navigation_msgs
    mcr_perception_msgs
    mcr_speech_msgs
    roscpp
    std_srvs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    mcr_navigation_msgs
    mcr_perception_msgs
    mcr_speech_msgs
    std_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(topic_to_service_node
  ros/src/topic_to_service_node.cpp
)
add_dependencies(topic_to_service_node  
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(topic_to_service_node
  ${catkin_LIBRARIES}
)


install(TARGETS topic_to_service_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
