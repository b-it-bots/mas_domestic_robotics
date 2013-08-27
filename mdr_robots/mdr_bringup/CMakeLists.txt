cmake_minimum_required(VERSION 2.8.3)
project(mdr_bringup)
find_package(catkin REQUIRED)

catkin_package()

install(FILES robot.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY robots/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY components/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY tools/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
