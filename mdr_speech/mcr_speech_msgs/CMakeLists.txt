cmake_minimum_required(VERSION 2.8.3)
project(mcr_speech_msgs)

find_package(catkin REQUIRED 
  COMPONENTS
    message_generation
    std_msgs
)

add_message_files(
  FILES
   RecognizedSpeech.msg  
)

add_service_files(
   FILES
   ChangeGrammar.srv  
   GetRecognizedSpeech.srv
   SetRecognitionConfidence.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)
