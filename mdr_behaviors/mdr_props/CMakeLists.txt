cmake_minimum_required(VERSION 2.8.3)
project(mdr_props)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
)

install(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)