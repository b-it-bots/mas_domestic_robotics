cmake_minimum_required(VERSION 2.8.3)
project(mdr_actions)

find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    actionlib_msgs
    genmsg
    message_generation
    roslint
    rospy
)

add_action_files(
  DIRECTORY
    ros/action
  FILES
    PerceiveTable.action
    MoveBaseSafe.action
)


catkin_python_setup()

generate_messages(DEPENDENCIES
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    mcr_perception_msgs
    mcr_states
    message_runtime
    std_msgs
)

roslint_python()

### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch)
endif()


### INSTALLS
install(PROGRAMS
  ros/scripts/perceive_table
  ros/scripts/move_base_safe
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
