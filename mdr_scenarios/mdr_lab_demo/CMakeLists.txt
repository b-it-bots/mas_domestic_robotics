cmake_minimum_required(VERSION 2.8.3)
project(mdr_lab_demo)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    mcr_body_detection_3d
    mcr_default_env_config
    mcr_leg_detector
    mcr_object_categorization
    mcr_object_recognition
    mcr_people_tracker
    mcr_virtual_2d_scanner
    mdr_2dnav
    mdr_arm_navigation
    mdr_arm_pointing
    mdr_bottle_weight
    mdr_bringup
    mdr_follow_person
    mdr_haptic
    mdr_manipulation_launch
    mdr_mockup_nodes
    mdr_node_status
    mdr_simple_pick
    mdr_table_cleaning
)

install(PROGRAMS
  ros/scripts/lab_demo
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)


