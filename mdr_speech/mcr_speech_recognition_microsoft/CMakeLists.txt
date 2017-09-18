cmake_minimum_required(VERSION 2.8.3)
project(mcr_speech_recognition_microsoft)

### The code only compiles under windows
if(WIN32)

  set(EXECUTABLE_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/bin) #executables
  add_definitions( "/W3 /D_CRT_SECURE_NO_WARNINGS /wd4251 /wd4275 /wd4099  /DROS_BUILD_SHARED_LIBS=1" )
  
  include_directories(
    common/include
    C:/opt/rosdeps/hydro/x86/include
    C:/opt/ros/hydro/x86/include
    C:/catkin_ws/devel/include
  )

  link_directories(
    C:/opt/ros/hydro/x86/lib
    C:/opt/rosdeps/hydro/x86/lib 
    build/Release
  )

  link_libraries(
    roscpp
    rosconsole
    rostime
    roscpp_serialization
  )

  add_library(speech_recognition_microsoft
    common/src/speech_recognition.cpp
    common/src/grammar_parser.cpp
  )

  add_library(speech_recognition_tools
    common/src/string_operations.cpp
    common/src/config_file_reader.cpp
  )

  add_executable(speech_recognition_node
    ros/src/speech_recognition_node.cpp
  )
  target_link_libraries(speech_recognition_node
    speech_recognition_microsoft
    speech_recognition_tools
    boost_serialization-vc-mt-1_47
  )

  #add_executable(speech_recognition_test 
  #  ros/src/speech_recognition_test.cpp
  #)
  #target_link_libraries(speech_recognition_test
  #  speech_recognition_microsoft
  #  speech_recognition_tools
  #  boost_serialization-vc-mt-1_47
  #)

endif(WIN32)
