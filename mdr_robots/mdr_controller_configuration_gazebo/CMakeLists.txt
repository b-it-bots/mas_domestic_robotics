cmake_minimum_required(VERSION 2.8.3)
project(mdr_controller_configuration_gazebo)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    rostopic
    joint_trajectory_action
    pr2_controller_manager
    katana_arm_gazebo
 )

install(DIRECTORY ros/launch/
  DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch"
)

install(DIRECTORY ros/controller/
  DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/controller"
)
