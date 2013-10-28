cmake_minimum_required(VERSION 2.8.3)
project(mdr_2dnav)
find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    cob_navigation_global    
    map_server
    mcr_default_env_config
    mcr_navigation_tools
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
