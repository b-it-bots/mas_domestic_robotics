cmake_minimum_required(VERSION 2.8.3)
project(mdr_manipulation_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  actionlib_msgs
  geometry_msgs
  mas_perception_msgs
)

add_service_files(FILES
  GetBottleState.srv
  Grasp.srv
  UpdatePlanningScene.srv
)

generate_messages(DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  mas_perception_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    mas_perception_msgs
)
