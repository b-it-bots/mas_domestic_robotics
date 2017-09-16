cmake_minimum_required(VERSION 2.8.3)
project(mdr_behavior_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    geometry_msgs
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
