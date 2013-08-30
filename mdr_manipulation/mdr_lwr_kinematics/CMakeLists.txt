cmake_minimum_required(VERSION 2.8.3)
project(mdr_lwr_kinematics)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    moveit_core
    pluginlib
)

include_directories(
  ros/include
  ros/include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES
  DEPENDS
    roscpp
    moveit_core
    pluginlib
)


add_library(lwr_ikfast_moveit_kinematics
  ros/src/lwr_ikfast_moveit_plugin.cpp
)

target_link_libraries(lwr_ikfast_moveit_kinematics
  ${catkin_LIBRARIES}
)


install(
  TARGETS lwr_ikfast_moveit_kinematics
  LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
)

install(
  DIRECTORY ros/include
  DESTINATION include
)

install(
  FILES kinematics_plugins.xml
  DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}"
)
