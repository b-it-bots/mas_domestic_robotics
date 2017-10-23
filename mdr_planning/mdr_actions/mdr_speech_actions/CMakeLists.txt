cmake_minimum_required(VERSION 2.8.3)
project(mdr_speech_actions)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  genmsg
  message_generation
  roslint
  rospy
)

catkin_python_setup()

add_action_files(
  FILES
  Listen.action
  Answer.action
  Ask.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)
