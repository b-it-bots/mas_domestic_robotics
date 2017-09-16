cmake_minimum_required(VERSION 2.8.3)
project(mdr_robot_inspection_test)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    rospy
    mdr_move_base_safe
    mdr_bringup
    mdr_common_states
    rospy
    smach
    smach_ros
)

### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch pc1:=localhost pc2:=localhost)
endif()

### INSTALLS
install(PROGRAMS
  ros/scripts/robot_inspection_test
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/ros/scripts
)
install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
