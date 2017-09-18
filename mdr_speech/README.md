# mdr_speech

This folder contains the speech recognition MS windows components
- SpeechRecognition is the SAPI 5.4 based API
- ros is a RosWindows wrapper for integration of the API
- test allows to run the recognition without ROS e.g. to test grammars

- it uses the brsu_sppech_common folder for certain string operations and the config file reader

### Build Requirements
- Windows 7
- Visual Studio 2008 or greater (Not express edition since COM objects are used)
- ROSWindows 1.1.0 (http://www.servicerobotics.eu/index.php?id=37)


### Recompile ROS messages and Services
If you change the ROS Services or Messages, you have to recompile them in Linux since ROSWin has no facilities for doing that yet. The generated files can/must be used in windows
Linux: roscd brsu_srvs
Linux: Type "rosmake"
Linux: roscd brsu_msgs
Linux: Type "rosmake"
Linux: Commit in git
Windows: Pull in git


### Services
ChangeGrammar.srv in brsu_common/srvs
rosservice call /changeGrammar "grammarfile"		("Grammar file will be loaded relative to windows server /SpeechRecognition/grammars folder!")


### Topics
Publishes "RecognizedSpeech.msg" in brsu_common/msgs
Message contains:
- String: The full speech sentence that has been understood
- String: A keyword related to the sentence for the scheduler
- Int32: Confidence of the speech recognizer (1 = high, 0 = mid, -1 = low)
- String[]: all keywords identified in the last recognized speech

### Change or Enhance Grammars
Grammars can be tested by dragging and dropping the grammar file over the gc.bat in the grammar folder
All variables used in the programm and grammars are lower case and with underscore
Grammars can be changed accorsign to the official SAPI grammars docu: http://msdn.microsoft.com/en-us/library/ee125672(v=VS.85).aspx

ATTENTION:
To make it possible to reference other grammar files with the rule ref tag, a custom parser was developed (SpeechRecognition/src/GrammarParser.cpp) since the microsoft parser seems not to work
This parser loads a grammer, checks all ruleref tags for the URI and combines them into one single grammar file (tempGrammar.xml). This tempGrammar will exist after the
programm is excited and can be checked with the grammar compiler if any problems occured.
This has 2 implications:
- Only 1 rule ref per line is allowed
- the URL tag will be checked for files ONLY!

## Contributors
- Mike - **Original author**
