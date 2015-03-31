cmake_minimum_required(VERSION 2.8.3)
project(mdr_cob_head_kinematics)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rosunit
    pluginlib
    moveit_core
)

catkin_package(
  LIBRARIES
    head_kinematics_plugin
  CATKIN_DEPENDS
    roscpp
    rosunit
    pluginlib
    moveit_core
)

# build
include_directories(
  common/include
  ros/include
)

add_library(mdr_cob_head_kinematics
  common/src/head_kinematics.cpp
)

add_library(mdr_cob_head_kinematics_plugin
  ros/src/head_kinematics_plugin.cpp
  ${catkin_LIBRARIES}
)

target_link_libraries(mdr_cob_head_kinematics_plugin
  mdr_cob_head_kinematics
)

# install
install(TARGETS mdr_cob_head_kinematics_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS mdr_cob_head_kinematics
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# test
catkin_add_gtest(head_kinematics_test
  common/test/head_kinematics_test.cpp
)

target_link_libraries(head_kinematics_test
  mdr_cob_head_kinematics
  ${catkin_LIBRARIES}
)

catkin_add_gtest(head_kinematics_plugin_test
  ros/test/head_kinematics_plugin_test.cpp
)

target_link_libraries(head_kinematics_plugin_test
  ${catkin_LIBRARIES}
)
