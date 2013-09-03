cmake_minimum_required(VERSION 2.8.3)
project(mdr_behaviors_msgs)
#find_package(catkin REQUIRED COMPONENTS arm_navigation_msgs geometry_msgs message_generator roscpp rospy)

add_message_files(
   FILES
#   CleanTable.msg
)

generate_messages(
   DEPENDENCIES
#   arm_navigation_msgs  geometry_msgs
)


