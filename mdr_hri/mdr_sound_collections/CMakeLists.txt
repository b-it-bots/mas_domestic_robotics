cmake_minimum_required(VERSION 2.8.3)
project(mdr_sound_collections)

find_package(catkin REQUIRED
)


catkin_package(
)

install(DIRECTORY collections
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

