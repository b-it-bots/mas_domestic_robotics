cmake_minimum_required(VERSION 2.8.3)
project(mdr_hri_behaviours)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslint
  smach
  message_generation
  mas_execution_manager
  mdr_rosplan_interface
)

find_package(std_msgs REQUIRED)

roslint_python()
catkin_python_setup()

add_service_files(
  FILES
  Prompt.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS
   rospy
   smach
   mas_execution_manager
   mdr_rosplan_interface
   std_msgs 
   message_runtime
)



include_directories(
  ${catkin_INCLUDE_DIRS}
)
