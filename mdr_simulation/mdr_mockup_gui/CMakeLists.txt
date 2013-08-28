cmake_minimum_required(VERSION 2.8.3)
project(mdr_mockup_gui)

find_package(catkin REQUIRED COMPONENTS roscpp rospy sensor_msgs mcr_speech_msgs)

catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   sensor_msgs
   mcr_speech_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES package_example
#  CATKIN_DEPENDS roscpp rospy sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(package_example
#   src/${PROJECT_NAME}/package_example.cpp
# )

## Declare a cpp executable
# add_executable(package_example_node src/package_example_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(package_example_node package_example_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(package_example_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  ros/scripts/mdr_mockup_gui
  ros/src/mdr_mockup_gui/mdr_mockup_gui.py
  ros/scripts/mdr_mockup_speech_wrapper
  ros/src/mdr_mockup_gui/mdr_mockup_speech_wrapper.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
# install(TARGETS package_example package_example_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )






# cmake_minimum_required(VERSION 2.4.6)
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# # Set the build type.  Options are:
# #  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
# #  Debug          : w/ debug symbols, w/o optimization
# #  Release        : w/o debug symbols, w/ optimization
# #  RelWithDebInfo : w/ debug symbols, w/ optimization
# #  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
# #set(ROS_BUILD_TYPE RelWithDebInfo)

# rosbuild_init()

# #set the default path for built executables to the "bin" directory
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# #set the default path for built libraries to the "lib" directory
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# #uncomment if you have defined messages
# #rosbuild_genmsg()
# #uncomment if you have defined services
# #rosbuild_gensrv()

# #common commands for building c++ executables and libraries
# #rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
# #target_link_libraries(${PROJECT_NAME} another_library)
# #rosbuild_add_boost_directories()
# #rosbuild_link_boost(${PROJECT_NAME} thread)
# rosbuild_add_executable(brsu_speech_recognition_sim ros/nodes/brsu_speech_recognition_sim.cpp)
# rosbuild_add_executable(brsu_speech_synthesis_sim ros/nodes/brsu_speech_synthesis_sim.cpp)
# rosbuild_add_executable(brsu_face_recognition_sim ros/nodes/brsu_face_recognition_sim.cpp)
# rosbuild_add_executable(brsu_door_status_node_sim ros/nodes/brsu_door_status_node_sim.cpp)

# rosbuild_add_executable(brsu_object_recognition_mockup ros/nodes/brsu_object_recognition_mockup.cpp)
# rosbuild_add_executable(brsu_object_segmentation_mockup ros/nodes/brsu_object_segmentation_mockup.cpp)
# rosbuild_add_executable(brsu_leg_detector_sim ros/nodes/brsu_leg_detection_sim.cpp)
# rosbuild_add_executable(brsu_people_map_sim ros/nodes/brsu_people_map_sim.cpp)
# rosbuild_add_executable(brsu_people_detection_mockup ros/nodes/brsu_people_detection_mockup.cpp)
# rosbuild_add_executable(brsu_arm_controller_mockup ros/nodes/brsu_arm_controller_mockup.cpp)
# rosbuild_add_executable(objectCandidateExtraction3D_mockup ros/nodes/objectCandidateExtraction3D_mockup.cpp)
# rosbuild_add_executable(brsu_haptic_mockup ros/nodes/brsu_haptic_mockup.cpp)
# rosbuild_add_executable(brsu_arm_pointing_mockup ros/nodes/brsu_arm_pointing.cpp)
# rosbuild_add_executable(brsu_clean_table_mockup ros/nodes/brsu_clean_table_mockup.cpp)
# rosbuild_add_executable(brsu_base_controller_mockup ros/nodes/brsu_base_controller_mockup.cpp)
# rosbuild_add_executable(brsu_torso_controller_mockup ros/nodes/brsu_torso_controller_mockup.cpp)
# rosbuild_add_executable(brsu_tray_controller_mockup ros/nodes/brsu_tray_controller_mockup.cpp)
# rosbuild_add_executable(brsu_head_controller_mockup ros/nodes/brsu_head_controller_mockup.cpp)
# rosbuild_add_executable(brsu_sdh_controller_mockup ros/nodes/brsu_sdh_controller_mockup.cpp)

# rosbuild_add_executable(brsu_simple_pick_mockup ros/nodes/brsu_simple_pick_mockup.cpp)
# rosbuild_add_executable(brsu_simple_place_mockup ros/nodes/brsu_simple_place_mockup.cpp)
# rosbuild_add_executable(brsu_arm_interaction_mockup ros/nodes/brsu_arm_interaction_mockup.cpp)




#target_link_libraries(example ${PROJECT_NAME})
