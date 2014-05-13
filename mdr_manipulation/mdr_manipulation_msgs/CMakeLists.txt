cmake_minimum_required(VERSION 2.8.3)
project(mdr_manipulation_msgs)

find_package(catkin REQUIRED
  COMPONENTS message_generation geometry_msgs
)

add_service_files(FILES
  GetBottleState.srv
  GetWrench.srv
  Grasp.srv
)

generate_messages(DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
)
