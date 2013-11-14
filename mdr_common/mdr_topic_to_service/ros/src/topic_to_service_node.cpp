#include <geometry_msgs/PoseWithCovariance.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <mcr_speech_msgs/GetRecognizedSpeech.h>
#include <mcr_speech_msgs/RecognizedSpeech.h>


#include <mcr_navigation_msgs/GetBasePose.h>

#include <mcr_perception_msgs/GetFaceName.h>
#include <mcr_perception_msgs/GetObjectList.h>
#include <mcr_perception_msgs/GetPersonList.h>
#include <mcr_perception_msgs/FaceList.h>
#include <mcr_perception_msgs/IsPersonInFront.h>
#include <mcr_perception_msgs/Object.h>
#include <mcr_perception_msgs/ObjectList.h>
#include <mcr_perception_msgs/PersonList.h>





#include <string>

geometry_msgs::PoseWithCovariance lastBasePoseInMap;
mcr_speech_msgs::RecognizedSpeech lastSpeechCommand;
mcr_perception_msgs::PersonList last_observed_persons;
mcr_perception_msgs::PersonList last_observed_legs;
bool isPlattformMoving = false;
std::string lastFaceName;

mcr_perception_msgs::ObjectList objectList;
bool isPersonInFrontValue = false;

mcr_perception_msgs::ObjectList object_categorization_objectList;

bool clearLastGetRecognizedSpeech()
{
  lastSpeechCommand.keyword = "no_speech";
  lastSpeechCommand.understood_phrase = "";
  lastSpeechCommand.confidence = 0;
  lastSpeechCommand.keyword_list.clear();
  lastSpeechCommand.confidence_list.clear();
  return true;
}



void GetRecognizedSpeechCallback(const mcr_speech_msgs::RecognizedSpeech& data)
{
  ROS_DEBUG("recognised speech");
  lastSpeechCommand = data;
}

bool lastSpeechCommandCallback(mcr_speech_msgs::GetRecognizedSpeech::Request  &req,	mcr_speech_msgs::GetRecognizedSpeech::Response &res )
{
  ROS_DEBUG("returning last recognised speech");
  
  res.keyword = lastSpeechCommand.keyword;
  res.understood_phrase = lastSpeechCommand.understood_phrase;
  res.confidence = lastSpeechCommand.confidence;
  res.keyword_list = lastSpeechCommand.keyword_list;
  res.confidence_list = lastSpeechCommand.confidence_list;
  clearLastGetRecognizedSpeech();
  
  return true;
}

//notification handler on new messages
void bodyDetectionCallback(const mcr_perception_msgs::PersonList& data)
{
	if (data.persons.size() > 0) {
		last_observed_persons = data;
	}
}

bool clearPersonList(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
	last_observed_persons.persons.clear();
	return true;
}

bool getBodyDetectionList(mcr_perception_msgs::GetPersonList::Request& request, mcr_perception_msgs::GetPersonList::Response& response)
{
	response.person_list = last_observed_persons;

	return true;
}

void legDetectionCallback(const mcr_perception_msgs::PersonList& data)
{
	last_observed_legs = data;
}

bool getLegDetectionList(mcr_perception_msgs::GetPersonList::Request& request, mcr_perception_msgs::GetPersonList::Response& response)
{
	response.person_list = last_observed_legs;
	last_observed_legs.persons.clear();

	return true;
}

bool clearLastGetRecognizedSpeechCallback(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
{
  clearLastGetRecognizedSpeech();
  return true;
}


void isPersonInFrontCallback(const mcr_perception_msgs::FaceList& data)
{
  if(data.num_faces > 0){
  	isPersonInFrontValue = true;
  //	ROS_INFO("is Person In Front callback");
  }else{
  	isPersonInFrontValue = false;
  }
}

bool isPersonInFront(mcr_perception_msgs::IsPersonInFront::Request& request, mcr_perception_msgs::IsPersonInFront::Response& response){
	response.value = isPersonInFrontValue;
	isPersonInFrontValue = false;
	//ROS_INFO("is Person In Front service call");
	return true;
}

void lastRecognizedFaces(const mcr_perception_msgs::FaceList& data)
{
  if(data.faces.size() > 0){
    lastFaceName = data.faces[0].name;
  }else{
    lastFaceName = "";
  }
}

bool getLastFaceName(mcr_perception_msgs::GetFaceName::Request& request, mcr_perception_msgs::GetFaceName::Response& response){

  response.value = lastFaceName;
  lastFaceName = "";
	return true;
}

void basePoseCallback(const geometry_msgs::PoseWithCovariance& data)
{
	lastBasePoseInMap = data;
}

bool getBasePose(mcr_navigation_msgs::GetBasePose::Request& request, mcr_navigation_msgs::GetBasePose::Response& response)
{
	geometry_msgs::Pose pose;

	pose.position = lastBasePoseInMap.pose.position;
	pose.orientation = lastBasePoseInMap.pose.orientation;

	response.pose = pose;
	return true; 
}


void objectRecognitionResponseCallback(const mcr_perception_msgs::ObjectListPtr& objectPoseList)
{

	objectList = *objectPoseList;
	
}


bool getObjectPoseListSrv(mcr_perception_msgs::GetObjectList::Request &req, mcr_perception_msgs::GetObjectList::Response &res)
{
    res.objects = objectList.objects;

    objectList.objects.clear();

    return true;
}

void object_categorizationCallback(const mcr_perception_msgs::ObjectListPtr& objectPoseList)
{
	object_categorization_objectList = *objectPoseList;	
}


bool get_object_categorization(mcr_perception_msgs::GetObjectList::Request &req, mcr_perception_msgs::GetObjectList::Response &res)
{
    res.objects = object_categorization_objectList.objects;
    
	object_categorization_objectList.objects.clear();

	return true;
}

bool clearStoredInfosCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_INFO("clearing stored information");

	geometry_msgs::PoseWithCovariance dummy1;
	lastBasePoseInMap = dummy1;
	
	mcr_speech_msgs::RecognizedSpeech dummy2;
	lastSpeechCommand = dummy2;

	mcr_perception_msgs::PersonList dummy3;
	last_observed_persons = dummy3;
	
	isPlattformMoving = false;

	lastFaceName="";

	mcr_perception_msgs::ObjectList dummy4;
	objectList = dummy4;

	isPersonInFrontValue = false;

	mcr_perception_msgs::ObjectList dummy5;
	object_categorization_objectList = dummy5;

	clearLastGetRecognizedSpeech();

	return true;
}




int main(int argc, char **argv)
{

  ros::init(argc, argv, "topic_to_service");
  ros::NodeHandle n("~");
 	
  ros::ServiceServer service_clear = n.advertiseService("clear_stored_infos", clearStoredInfosCallback); 	
 	
  ros::Subscriber sub1 = n.subscribe("/mcr_speech_speech_recognition/recognized_speech", 1000, GetRecognizedSpeechCallback);
  ros::ServiceServer service1 = n.advertiseService("/mcr_speech_speech_recognition/get_last_recognized_speech", lastSpeechCommandCallback);
  ros::ServiceServer service2 = n.advertiseService("/mcr_speech_speech_recognition/clear_last_recognized_speech", clearLastGetRecognizedSpeechCallback);
  
  ros::Subscriber sub2 = n.subscribe("/mcr_perception/face_recognition/is_person_in_front", 1, isPersonInFrontCallback);
  ros::ServiceServer service3 = n.advertiseService("/mcr_perception/face_recognition/is_person_in_front", isPersonInFront);

  ros::Subscriber facerecsub2 = n.subscribe("/mcr_perception/face_recognition/recognized_faces", 1, lastRecognizedFaces);
  ros::ServiceServer getLastFaceNameService = n.advertiseService("/mcr_perception/face_recognition/get_last_face_name", getLastFaceName);
  
  ros::Subscriber map_pose_sub = n.subscribe("/base_pose", 1000, basePoseCallback);
  ros::ServiceServer service6 = n.advertiseService("/base_controller/get_base_pose", getBasePose);
  
  ros::Subscriber subObjectRecognitionResponse = n.subscribe("/moped/camera_poses", 1, objectRecognitionResponseCallback);
  ros::ServiceServer srvObjectRecognition = n.advertiseService("/mdr_perception/object_recognition/get_object_list", getObjectPoseListSrv);

  ros::Subscriber sub_body_detections = n.subscribe("/mdr_perception/body_detection_3d/people_positions", 1, bodyDetectionCallback);
  ros::ServiceServer srv_clear_person_list = n.advertiseService("/mdr_perception/body_detection_3d/clear_person_list", clearPersonList);
  ros::ServiceServer srv_body_list = n.advertiseService("/mdr_perception/body_detection_3d/get_person_list", getBodyDetectionList);

  ros::Subscriber sub_leg_detections = n.subscribe("/mdr_perception/leg_detection/leg_positions", 1, legDetectionCallback);
  ros::ServiceServer srv_leg_list = n.advertiseService("/mdr_perception/leg_detection/get_person_list", getLegDetectionList);

  ros::Subscriber object_categorization_sub = n.subscribe("/mdr_perception/object_categorization/categorized_objects", 1, object_categorizationCallback);
  ros::ServiceServer mdr_object_categorization_service = n.advertiseService("/mdr_perception/object_categorization/categorized_objects", get_object_categorization);


  ros::spin();

  return 0;
}

