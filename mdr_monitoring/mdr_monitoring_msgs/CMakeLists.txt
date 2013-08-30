cmake_minimum_required(VERSION 2.8.3)
project(mdr_monitoring_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(
  FILES
  AudioEndpoint.msg  
  Endpoints.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
