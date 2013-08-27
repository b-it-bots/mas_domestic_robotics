cmake_minimum_required(VERSION 2.8.3)
project(mdr_calibration_data)
find_package(catkin REQUIRED)

set(ROBOTS
  volksbot
)

catkin_package()

foreach(robot ${ROBOTS})
  install(
    DIRECTORY ${robot}/
    DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}/${robot}"
  )
endforeach()
