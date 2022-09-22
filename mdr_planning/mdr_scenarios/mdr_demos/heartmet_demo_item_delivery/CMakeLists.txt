cmake_minimum_required(VERSION 2.8.3)
project(heartmet_demo_item_delivery)

find_package(catkin REQUIRED COMPONENTS
  rospy
  smach_ros
  smach
  actionlib
  mongodb_store
  mdr_rosplan_interface
  mas_execution_manager
  mas_hsr_move_base_action
  mas_hsr_move_arm_action
  mas_hsr_pickup_action
  mas_hsr_place_action
  mas_hsr_perceive_plane_action
  mdr_demo_simple_pick_and_place
  mdr_demo_context_aware_hand_over
  mas_hsr_find_people
  mas_hsr_hand_over_action
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    smach_ros
    smach
    actionlib
    mongodb_store
    mdr_rosplan_interface
    mas_execution_manager
    mas_hsr_move_base_action
    mas_hsr_move_arm_action
    mas_hsr_pickup_action
    mas_hsr_place_action
    mas_hsr_perceive_plane_action
    mdr_demo_simple_pick_and_place
    mdr_demo_context_aware_hand_over
    mas_hsr_find_people
    mas_hsr_hand_over_action
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

