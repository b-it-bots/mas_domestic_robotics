cmake_minimum_required(VERSION 2.8.3)
project(mdr_bringup_sim)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS cob_default_robot_config cob_gazebo gazebo_ros mcr_default_env_config mcr_gazebo_worlds mdr_hardware_config mdr_default_robot_config mdr_controller_configuration_gazebo 
)
  
install(FILES robot.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)

install(DIRECTORY robots/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/robots
)
