cmake_minimum_required(VERSION 2.8.3)
project(mdr_environments)

find_package(catkin REQUIRED)

catkin_package()

### TESTS
set(env_list
  brsu-c069
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  foreach(env ${env_list})
    roslaunch_add_file_check(upload_param.launch robot_env:=${env})
  endforeach()
endif()

### INSTALLS
install(FILES upload_param.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/upload_param.launch
)

install(DIRECTORY brsu-c069/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/brsu-c069/
)
