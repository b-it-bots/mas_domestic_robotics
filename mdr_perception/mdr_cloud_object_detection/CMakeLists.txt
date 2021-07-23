if($ENV{ROS_DISTRO} STREQUAL "noetic")
  cmake_minimum_required(VERSION 3.0.2)
  add_compile_options(-std=c++14 -Wno-format-security)
else()
  cmake_minimum_required(VERSION 2.8.3)
  add_compile_options(-std=c++11 -Wno-format-security)
endif()
project(mdr_cloud_object_detection)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  actionlib
  dynamic_reconfigure
  tf2_geometry_msgs
  pcl_ros
  roscpp
  mas_navigation_tools
  mas_perception_msgs
  mas_perception_libs
)

find_package(PCL 1.7 REQUIRED)
find_package(Python REQUIRED)
find_package(Boost REQUIRED COMPONENTS python${Python_VERSION_MAJOR}${Python_VERSION_MINOR})

generate_dynamic_reconfigure_options(
  ros/config/ObjectDetection.cfg
)

catkin_package(
  INCLUDE_DIRS
    ros/include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    dynamic_reconfigure
    mas_navigation_tools
)

###########
## Build ##
###########
include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

###################################
# Node for object detection
add_executable(cloud_object_detection
  ros/src/CropBoxVoxelFilter.cpp
  ros/src/CloudObjectDetection.cpp
  ros/src/main.cpp
)

add_dependencies(cloud_object_detection ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(cloud_object_detection
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS cloud_object_detection
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
