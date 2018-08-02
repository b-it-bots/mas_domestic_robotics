cmake_minimum_required(VERSION 2.8.3)
project(mdr_speech_pose_saver)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
)

catkin_package()
catkin_python_setup()

install(PROGRAMS
  ros/scripts/speech_pose_saver_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/ros/scripts
)

install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
