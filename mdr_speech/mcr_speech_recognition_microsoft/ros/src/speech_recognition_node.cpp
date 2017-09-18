#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <ros/network.h>

#include <iostream>
#include <fstream>
#include <string>

#include <mcr_speech_msgs/ChangeGrammar.h> //include service definition
#include <mcr_speech_msgs/SetRecognitionConfidence.h> //include service definition
#include <mcr_speech_msgs/RecognizedSpeech.h> //include message definition

#include "mcr_speech_recognition_microsoft/speech_recognition.h" //include speech
#include "mcr_speech_recognition_microsoft/string_operations.h"
#include "mcr_speech_recognition_microsoft/config_file_reader.h"

SpeechRecognition* recognizer;
std::string grammarFile = "robot_inspection.xml";
std::string rosMaster = "";
std::string localIP = "";
std::string grammarFolder = "";
std::string mediaFiolder = "";
std::string acknowledgeSound = "";

std::string understoodPhrase;
std::string matchedKeyword;
float resultConfidence;
std::vector<std::string> resultKeywordList;
std::vector<float> resultConfidenceList;

std::string CONFIG_FILE = "config.cfg";

#define DEFAULT_GRAMMAR_FILE "robot_inspection.xml"
#define DEFAULT_ROS_MASTER "http://cob3-1-pc1:11311/"
#define DEFAULT_LOCALIP "cob3-1-pc3"
#define DEFAULT_GRAMMAR_FOLDER "./grammars"
#define DEFAULT_MEDIA_FOLDER "./media"
#define DEFAULT_ACKNOWLEDGE_SOUND_FILE "beep1.wav"

bool active = true;
/**

 * This tutorial demonstrates simple sending of messages over the ROS system.
 */


bool changeGrammar(mcr_speech_msgs::ChangeGrammar::Request  &req,
                   mcr_speech_msgs::ChangeGrammar::Response &res)
{
    recognizer->loadGrammar(req.grammar);
    res.result = 0;
    return true;
}

bool changeRecognitionThreshold(mcr_speech_msgs::SetRecognitionConfidence::Request  &req,
                                mcr_speech_msgs::SetRecognitionConfidence::Response &res)
{
    recognizer->setConfidenceThreshold(req.threshold);
    return true;
}

//Initialize data required from file
void readConfigFile()
{
    ConfigFileReader reader;
    reader.setConfigFile(CONFIG_FILE);

    if (reader.isFileExisting())
    {
        rosMaster = reader.getROSMaster();
        localIP = reader.getLocalIP();
        grammarFolder = reader.getGrammarFolder();
        mediaFiolder = reader.getMediaFolder();
        acknowledgeSound = reader.getAcknowledgeSoundFile();
        grammarFile = reader.getGrammar();
    }
    else
    {
        std::cout << "!!!Config file not found, using default values!!!" << std::endl;
        rosMaster = DEFAULT_ROS_MASTER;
        localIP = DEFAULT_LOCALIP;
        grammarFolder = DEFAULT_GRAMMAR_FOLDER;
        mediaFiolder = DEFAULT_MEDIA_FOLDER;
        acknowledgeSound = DEFAULT_ACKNOWLEDGE_SOUND_FILE;
        grammarFile = DEFAULT_GRAMMAR_FILE;
    }
}

int main(int argc, char **argv)
{
    try
    {
        recognizer = new SpeechRecognition();
        //parse arguments, argv[1] should be config file path
        if (argc > 1)
        {
            CONFIG_FILE = argv[1];
        }
        else
        {
            std::cout << "*** No config file passed as argument, please use \"speech_recognition_node.exe configfile.cfg\" !!!" << std::endl << "Searching in current folder..." << std::endl;
        }

        readConfigFile();

        ros::init(argc, argv, "mcr_speech_recognition");
        ros::NodeHandle n("~");

        ros::ServiceServer serviceChangeGrammar = n.advertiseService("change_grammar", changeGrammar);
        ros::ServiceServer serviceChangeRecognitionThreshold = n.advertiseService("change_recognition_threshold", changeRecognitionThreshold);

        ros::Publisher pub = n.advertise<mcr_speech_msgs::RecognizedSpeech>("recognized_speech", 1000);

        ros::Rate r(10);

        //load grammar from config
        recognizer->setAcknowledgeSoundFileName(acknowledgeSound);
        recognizer->setGrammarFolderName(grammarFolder);
        recognizer->setMediaFolderName(mediaFiolder);
        recognizer->initSpeech(grammarFile);

        ROS_INFO("Speech Recognition started");
        while (n.ok())
        {
            mcr_speech_msgs::RecognizedSpeech msg;

            std::stringstream ss;
            if (active)
            {
                recognizer->CheckForPhrase(&understoodPhrase, &matchedKeyword, &resultConfidence, &resultKeywordList, &resultConfidenceList);

                if (matchedKeyword != recognizer->noSpeechKeyword)
                {
                    msg.understood_phrase = understoodPhrase.c_str();
                    msg.keyword = matchedKeyword.c_str();
                    msg.confidence = resultConfidence;
                    msg.keyword_list = resultKeywordList;
                    msg.confidence_list = resultConfidenceList;

                    ROS_INFO("Phrase: %s", understoodPhrase.c_str());
                    ROS_INFO("MainKeyword: %s", matchedKeyword.c_str());
                    ROS_INFO("Confidence: %f", resultConfidence);
                    for (unsigned int i = 0; i < resultKeywordList.size(); i++)
                    {
                        ROS_INFO("All_Keywords %s", resultKeywordList[i].c_str());
                        ROS_INFO("Confidence %f", resultConfidenceList[i]);
                    }

                    pub.publish(msg);
                }
            }
            ros::spinOnce();
            r.sleep();
        }

    }
    catch (std::exception &e)
    {
        std::cout << e.what() << std::endl;
    }
    catch (char *str)
    {
        std::cout << "Caught some other exception: " << str << std::endl;
    }
    catch (...)
    {
        std::cout << "unhandled exception" << std::endl;
    }
    delete recognizer;

    return 0;
}

