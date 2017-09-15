cmake_minimum_required(VERSION 2.8.3)
project(mdr_behaviors_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    geometry_msgs
    #arm_navigation_msgs
    #geometry_msgs
)

add_message_files(
  FILES
#   CleanTable.msg
)

add_service_files(
  FILES
    Pickup.srv
    Place.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
)
