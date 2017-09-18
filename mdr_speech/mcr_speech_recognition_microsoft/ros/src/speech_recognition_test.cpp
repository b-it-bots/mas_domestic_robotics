/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Initial WIN32 port by Hozefa Indorewala, Robotics Equipment Corporation GmbH, www.servicerobotics.eu */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/network.h>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

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
            std::cout << "*** No config file passed as argument, please use \"SpeechSynthesisROS.exe configfile.cfg\" !!!" << std::endl << "Searching in current folder..." << std::endl;
        }

        readConfigFile();

        //set master URI
//  ros::master::setURI(rosMaster);
//  ros::master::setURI("http://10.20.121.61:11311/");
//  std::cout<< "Connect to ROS master: " << rosMaster << std::endl;

        //set ros ip
//  ros::network::setHost(localIP);
        // std::cout<< "local ROS IP: " << localIP << std::endl;


        /**
        * The ros::init() function needs to see argc and argv so that it can perform
        * any ROS arguments and name remapping that were provided at the command line. For programmatic
        * remappings you can use a different version of init() which takes remappings
        * directly, but for most command-line programs, passing argc and argv is the easiest
        * way to do it.  The third argument to init() is the name of the node.
        *
        * You must call one of the versions of ros::init() before using any other
        * part of the ROS system.
        */
//  ros::init(argc, argv, "brsu_speech_recognition");

        /**
         * NodeHandle is the main access point to communications with the ROS system.
         * The first NodeHandle constructed will fully initialize this node, and the last
         * NodeHandle destructed will close down the node.
         */
// ros::NodeHandle n;
//
//  ros::ServiceServer serviceChangeGrammar = n.advertiseService("/brsu_speech_recognition/change_grammar", changeGrammar);
//  ros::ServiceServer serviceChangeRecognitionThreshold = n.advertiseService("/brsu_speech_recognition/change_recognition_threshold", changeRecognitionThreshold);

        /**
         * The advertise() function is how you tell ROS that you want to
         * publish on a given topic name. This invokes a call to the ROS
         * master node, which keeps a registry of who is publishing and who
         * is subscribing. After this advertise() call is made, the master
         * node will notify anyone who is trying to subscribe to this topic name,
         * and they will in turn negotiate a peer-to-peer connection with this
         * node.  advertise() returns a Publisher object which allows you to
         * publish messages on that topic through a call to publish().  Once
         * all copies of the returned Publisher object are destroyed, the topic
         * will be automatically unadvertised.
         *
         * The second parameter to advertise() is the size of the message queue
         * used for publishing messages.  If messages are published more quickly
         * than we can send them, the number here specifies how many messages to
         * buffer up before throwing some away.
         */
//  ros::Publisher pub = n.advertise<mcr_speech_msgs::RecognizedSpeech>("/brsu_speech_recognition/recognized_speech", 1000);

        /**
         * A count of how many messages we have sent. This is used to create
         * a unique string for each message.
         */
        //ros::Rate r(10);

        //load grammar from config
        recognizer->setAcknowledgeSoundFileName(acknowledgeSound);
        recognizer->setGrammarFolderName(grammarFolder);
        recognizer->setMediaFolderName(mediaFiolder);
        recognizer->initSpeech(grammarFile);

        while (true)
        {
            /**
             * This is a message object. You stuff it with data, and then publish it.
             */
            mcr_speech_msgs::RecognizedSpeech msg;

            std::stringstream ss;

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
                ROS_INFO("\n");

                /**
                 * The publish() function is how you send messages. The parameter
                 * is the message object. The type of this object must agree with the type
                 * given as a template parameter to the advertise<>() call, as was done
                 * in the constructor above.
                 */
//      pub.publish(msg);
            }

            //  ros::spinOnce();
            //  r.sleep();
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

