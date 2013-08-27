cmake_minimum_required(VERSION 2.8.3)
project(mdr_hardware_config)

find_package(catkin REQUIRED)

set(ROBOTS
  volksbot
)

catkin_package(
  DEPENDS mdr_calibration_data mdr_description
)

foreach(robot ${ROBOTS})
  install(DIRECTORY ${robot}/
     DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}/${robot}"
  )
endforeach(robot)

install(DIRECTORY common/
  DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}/common"
)
