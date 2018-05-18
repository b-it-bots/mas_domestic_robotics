cmake_minimum_required(VERSION 2.8.3)
project(mdr_sound_communication)

find_package(catkin REQUIRED
  COMPONENTS
    roslint
    std_msgs
    dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
   ros/cfg/soundCommunication.cfg
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    rospkg
    std_msgs
    dynamic_reconfigure
)

### TESTS
roslint_python()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch)
endif()

install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(PROGRAMS
  ros/scripts/willow_sounds
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
