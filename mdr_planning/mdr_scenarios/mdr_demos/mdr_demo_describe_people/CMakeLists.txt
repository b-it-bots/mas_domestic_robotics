cmake_minimum_required(VERSION 2.8.3)
project(mdr_demo_describe_people)

find_package(catkin REQUIRED COMPONENTS
  roslint
  rospy
  smach_ros
  smach
  actionlib
  mas_execution_manager
  mdr_detect_person
  mdr_gender_recognition
  mdr_recognize_emotion_action
)

catkin_package(
  CATKIN_DEPENDS
    roslint
    rospy
    smach_ros
    smach
    actionlib
    mas_execution_manager
    mdr_detect_person
    mdr_gender_recognition
    mdr_recognize_emotion_action
)

catkin_python_setup()
roslint_python()

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
