#include <ros/ros.h>
#include <ros/network.h>
#include <std_msgs/Bool.h>
#include <windows.h>
#include <iostream>
#include <string>
#include <sapi.h>

#include <std_msgs/String.h>

#include <mcr_speech_recognition_microsoft/string_operations.h>

#define DEFAULT_ROS_MASTER "http://cob3-1-pc1:11311/"
#define DEFAULT_LOCALIP "cob3-1-pc3"

std::string rosMaster = "";
std::string localIP = "";
ISpVoice * pVoice = NULL;

ros::Publisher pubEvent;
std_msgs::String event_out;
/**
* Method to say something and publish it on ROS info
**/
void say(std::string data)
{
    data = StringOperations::replaceAll(data, "_", " ");

    ROS_INFO("Say: [%s]", data.c_str());
    std::wstring stemp = StringOperations::stdstring2LPW(data);

    pVoice->Speak(stemp.c_str(), 0, NULL);
    pVoice->WaitUntilDone(30000); //30sec timeout

    event_out.data = "e_done";
    pubEvent.publish(event_out);
}

void sayCallback(const std_msgs::String::ConstPtr &msg)
{
    say(msg->data);
}

bool initVoice()
{
    if (FAILED(::CoInitialize(NULL)))
        return FALSE;

    HRESULT hr = CoCreateInstance(CLSID_SpVoice, NULL, CLSCTX_ALL, IID_ISpVoice, (void **)&pVoice);
    if (FAILED(hr))
    {
        return FALSE;
    }

    return TRUE;
}

int main(int argc, char **argv)
{
    // setup speech synthesis
    SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS);
    initVoice();
    pVoice->Speak(L" ", 0, NULL);

    ros::init(argc, argv, "mcr_speech_synthesis");
    ros::NodeHandle n("~");

    ros::Subscriber subSay = n.subscribe("/say", 100, sayCallback);
    pubEvent = n.advertise<std_msgs::String>("event_out", 1);

    ROS_INFO("Speech Synthesis started");

    ros::spin();

    pVoice->Release();
    pVoice = NULL;

    ::CoUninitialize();

    return 0;
}
