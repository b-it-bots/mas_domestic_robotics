cmake_minimum_required(VERSION 2.8.3)
project(mcr_speech_synthesis)

### The code only compiles under windows
if(WIN32)

  set(EXECUTABLE_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/bin) #executables
  add_definitions( "/W3 /D_CRT_SECURE_NO_WARNINGS /wd4251 /wd4275 /wd4099  /DROS_BUILD_SHARED_LIBS=1" )
  
  include_directories(
    C:/opt/rosdeps/hydro/x86/include
    C:/opt/ros/hydro/x86/include
    C:/catkin_ws/devel/include
    ../mcr_speech_recognition_microsoft/common/include
  )

  link_directories(
    C:/opt/ros/hydro/x86/lib
    C:/opt/rosdeps/hydro/x86/lib
    ../mcr_speech_recognition_microsoft/build/Release
  )

  link_libraries(
    roscpp
    rosconsole
    rostime
    roscpp_serialization
  )

  add_executable(speech_synthesis_node 
    ros/src/speech_synthesis_node.cpp
  )
  target_link_libraries(speech_synthesis_node
    speech_recognition_tools
  )

endif()
