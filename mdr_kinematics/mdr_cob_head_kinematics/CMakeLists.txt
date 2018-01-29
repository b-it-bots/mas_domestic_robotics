cmake_minimum_required(VERSION 2.8.3)
project(mdr_cob_head_kinematics)
set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf
    pluginlib
    moveit_core
)

catkin_package(
  LIBRARIES
    head_kinematics_plugin
  CATKIN_DEPENDS
    roscpp
    pluginlib
    moveit_core
)

# build
include_directories(
  common/include
  ros/include
  ${catkin_INCLUDE_DIRS}
)

add_library(mdr_cob_head_kinematics
  common/src/head_kinematics.cpp
)

add_library(mdr_cob_head_kinematics_plugin
  ros/src/head_kinematics_plugin.cpp
)
target_link_libraries(mdr_cob_head_kinematics_plugin
  mdr_cob_head_kinematics
  ${catkin_LIBRARIES}
)

add_executable(look_at_point_node
  ros/src/look_at_point_node.cpp
)
target_link_libraries(look_at_point_node
  ${catkin_LIBRARIES}
)
add_dependencies(look_at_point_node ${catkin_EXPORTED_TARGETS})

# install
install(
  TARGETS
    mdr_cob_head_kinematics
    mdr_cob_head_kinematics_plugin
    look_at_point_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(DIRECTORY ros/config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/config
)

install(FILES kinematics_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# test
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rosunit REQUIRED)

  roslaunch_add_file_check(ros/launch)

  catkin_add_gtest(head_kinematics_plugin_test
    ros/test/head_kinematics_plugin_test.cpp
  )

  target_link_libraries(head_kinematics_plugin_test
    ${catkin_LIBRARIES}
  )
endif()
