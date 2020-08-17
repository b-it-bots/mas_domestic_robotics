cmake_minimum_required(VERSION 2.8.3)
project(mdr_util_register_person)

find_package(catkin REQUIRED COMPONENTS
  rospy
  mdr_detect_person
  mas_execution_manager
  mdr_perception_behaviours
  mas_knowledge_base
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    mdr_detect_person
    mas_execution_manager
    mdr_perception_behaviours
    mas_knowledge_base
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
