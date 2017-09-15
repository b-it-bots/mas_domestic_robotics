cmake_minimum_required(VERSION 2.8.3)
project(mdr_bringup)
find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    mcr_default_env_config
    mcr_teleop
    mdr_default_robot_config
    cob_bringup
    sicktoolbox_wrapper
    hokuyo_node
    robot_state_publisher
    openni2_launch
    tf
)

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
