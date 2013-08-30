cmake_minimum_required(VERSION 2.8.3)
project(mdr_emergency_stop_monitor)

find_package(catkin REQUIRED
  COMPONENTS
    std_msgs
    std_srvs
    cob_relayboard
    mcr_speech_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs std_srvs cob_relayboard mcr_speech_msgs
)

add_executable(mdr_emergency_stop_monitor
  ros/src/mdr_emergency_stop_monitor.cpp
)

target_link_libraries(mdr_emergency_stop_monitor
  ${catkin_LIBRARIES}
)

add_dependencies(mdr_emergency_stop_monitor ${catkin_EXPORTED_TARGETS})

install(TARGETS mdr_emergency_stop_monitor
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)