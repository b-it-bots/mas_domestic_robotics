cmake_minimum_required(VERSION 2.8.3)
project(mdr_volksbot_motorcontroller)

find_package(catkin REQUIRED
  COMPONENTS
     geometry_msgs
     nav_msgs
     pr2_msgs
     roscpp
     sensor_msgs
     tf
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs pr2_msgs sensor_msgs
)

include_directories(
  common/include
  common/include/${PROJECT_NAME} 
)

add_library(volksbot_motorcontroller 
  common/src/robot_factory.cpp
  common/src/robot_platform_volksbot_rt.cpp
  common/src/robot_platform_differential_drive.cpp
  common/src/timer.cpp
  common/src/com_adapter.cpp
  common/src/layer/communication_layer.cpp
  common/src/layer/translation_layer.cpp
  common/src/layer/vmc_api.cpp
  common/src/storage/motor.cpp
  common/src/storage/storage.cpp
  common/src/support/channel.cpp
  common/src/support/data.cpp
  common/src/support/error.cpp
  common/src/support/message.cpp
  common/src/support/multisend.cpp
  common/src/support/request.cpp
  common/src/support/send_two.cpp
  common/src/support/timestamp.cpp	
)

add_executable(volksbot_motorcontroller_node 
  ros/src/volksbot_motorcontroller_node.cpp
)
add_dependencies(volksbot_motorcontroller_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(volksbot_motorcontroller_node
  volksbot_motorcontroller  
  ${catkin_LIBRARIES}
)

install(TARGETS volksbot_motorcontroller volksbot_motorcontroller
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS volksbot_motorcontroller_node volksbot_motorcontroller_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)