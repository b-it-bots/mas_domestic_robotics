#!/usr/bin/python

######################### IMPORTS #########################
import rospy
import smach
import smach_ros
import moveit_commander

import std_msgs.msg
import std_srvs.srv
import actionlib_msgs.msg


import mcr_perception_msgs.srv

from mdr_common_states.common_states_speech import *


from simple_script_server import *
sss = simple_script_server()


class search_for_object(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['success', 'no_object'], 
									input_keys=['grasp_position'], 
									output_keys=['grasp_position'])
	
	
		self.object_list_srv = rospy.ServiceProxy('/mcr_perception/object_recognition/get_object_list', mcr_perception_msgs.srv.GetObjectList)
	
	def execute(self, userdata):
		rospy.wait_for_service('/mcr_perception/object_recognition/get_object_list', 10)

		resp = self.object_list_srv()
		 
		if (len(resp.objects) > 0):
			print resp.objects[0].name
			userdata.grasp_position = resp.objects[0].position
			print userdata.grasp_position
			return 'success'			
		else:
			return 'no_object'

class detect_object(smach.State):
	def __init__(self):	
		smach.State.__init__(self, outcomes=['success','failed','retry'], output_keys=['grasp_position'])
		self.find_object_srv = rospy.ServiceProxy('/mcr_perception/tabletop_segmentation/get_detected_objects', mcr_perception_msgs.srv.GetObjectList)
		self.arm = moveit_commander.MoveGroupCommander('arm')

	def execute(self, userdata):
		
		sss.move("head", "back_table", False)
		sss.move("torso", "extrem_back")
	
		self.arm.set_named_target("look_at_table")
		self.arm.go()

		sss.sleep(6)

		rospy.wait_for_service('/mcr_perception/tabletop_segmentation/get_detected_objects', 30)
		for i in range(5): 
			print "calling GetObjectCandidates3D service"
			resp = self.find_object_srv()
			# check if there are at least one objects and less than 5 objects
			if ((len(resp.pointCloudCentroids) <= 0) or (resp.bestPointCloudCentroidIndex >= len(resp.pointCloudCentroids))):
				print "no graspable objects found"
			else:	
				if (len(resp.pointCloudCentroids) == 0):
					return 'retry'
				elif len(resp.pointCloudCentroids) > 1:
					SAY("I see {0} objects".format(len(resp.pointCloudCentroids)))
				else:
					SAY("I see one object")
				userdata.grasp_position = resp.pointCloudCentroids[resp.bestPointCloudCentroidIndex]
				return 'success'
		SAY("The table is empty.")
		self.arm.set_named_target("folded")
		self.arm.go()
		sss.move("head","front_face",False)
		return 'failed'

class find_object_moped(smach.State):
	def __init__(self):	
		smach.State.__init__(self, outcomes=['success','failed'], input_keys=['object_name'], output_keys=['object_name', 'grasp_position'])
		self.find_object_srv = rospy.ServiceProxy('/mcr_perception/object_recognition/get_object_list', mcr_perception_msgs.srv.GetObjectList)
		self.object_recognition_start = rospy.ServiceProxy('/mcr_perception/object_recognition_height_based/start', std_srvs.srv.Empty)
		self.object_recognition_stop = rospy.ServiceProxy('/mcr_perception/object_recognition_height_based/stop', std_srvs.srv.Empty)
		self.arm = moveit_commander.MoveGroupCommander('arm')

	def execute(self, userdata):
		rospy.wait_for_service('/mcr_perception/object_recognition_height_based/start', 30)
		rospy.wait_for_service('/mcr_perception/object_recognition_height_based/stop', 30)
		self.object_recognition_start()
		
		sss.move("head", "back_table", False)

		self.arm.set_named_target("look_at_table")
		self.arm.go()

		sss.move("torso", "extrem_back")
		rospy.sleep(5)

		SAY('I am looking for the ' + userdata.object_name)
	#	rospy.sleep(3)
		
		rospy.wait_for_service('/mcr_perception/object_recognition/get_object_list', 30)
		for i in range(20): 
			print "calling /mcr_perception/object_recognition/get_object_list service"
			resp = self.find_object_srv()

			if (len(resp.objects) <= 0):
				print "no graspable objects found"
				rospy.sleep(1)
			else:
				seenObjectName = ''
				for recObject in resp.objects:
					print "object name: " + recObject.name
					seenObjectName = recObject.name
					if userdata.object_name == seenObjectName:
						break
				if userdata.object_name == seenObjectName:
					break
				
		if (len(resp.objects) <= 0):
			SAY("I could not find the " + userdata.object_name + ".")

	                sss.move("torso", "home")

			self.arm.set_named_target("folded")
			self.arm.go()
					
			sss.move("head","front_face",False)
			self.object_recognition_stop()
			return 'failed'

		SAY("I see ")
		for recObject in resp.objects:
			SAY(recObject.name + ", ")

		for recObject in resp.objects:
			print "object name: " + recObject.name
		#	print "object position: " + recObject.pose.pose.position
			if userdata.object_name == recObject.name:
				userdata.grasp_position = recObject.pose.pose.position
				#SAY("I will now grasp the " + userdata.object_name)
				self.object_recognition_stop()
				return 'success'

		SAY("I could not find the " + userdata.object_name + ".")
		self.arm.set_named_target("folded")
		self.arm.go()
				
		sss.move("head","front_face",False)
		self.object_recognition_stop()
		return 'failed'
		
		
class find_one_known_object(smach.State):
	def __init__(self):	
		smach.State.__init__(self, outcomes=['success','failed'], input_keys=['object_name'], output_keys=['object_name', 'grasp_position'])
		self.find_object_srv = rospy.ServiceProxy('/mcr_perception/object_recognition/get_object_list', mcr_perception_msgs.srv.GetObjectList)
		self.object_recognition_start = rospy.ServiceProxy('/mcr_perception/object_recognition_height_based/start', std_srvs.srv.Empty)
		self.object_recognition_stop = rospy.ServiceProxy('/mcr_perception/object_recognition_height_based/stop', std_srvs.srv.Empty)
		self.arm = moveit_commander.MoveGroupCommander('arm')

	def execute(self, userdata):
		rospy.wait_for_service('/mcr_perception/object_recognition_height_based/start', 30)
		rospy.wait_for_service('/mcr_perception/object_recognition_height_based/stop', 30)
		self.object_recognition_start()
		
		sss.move("head", "back_table", False)
		self.arm.set_named_target("look_at_table")
		self.arm.go()
		sss.move("torso", "extrem_back")
		rospy.sleep(5)

		SAY('I am looking for the ' + userdata.object_name)
	#	rospy.sleep(3)
		
		rospy.wait_for_service('/mcr_perception/object_recognition/get_object_list', 30)
		for i in range(20): 
			print "calling /mcr_perception/object_recognition/get_object_list service"
			resp = self.find_object_srv()

			if (len(resp.objects) <= 0):
				print "no graspable objects found"
				rospy.sleep(1)
			else:
				seenObjectName = ''
				for recObject in resp.objects:
					print "object name: " + recObject.name
					seenObjectName = recObject.name
					if userdata.object_name == seenObjectName:
						break
				if userdata.object_name == seenObjectName:
					break
				
		if (len(resp.objects) <= 0):
			SAY("I could not find the " + userdata.object_name + ".")
			self.arm.set_named_target("folded")
			self.arm.go()
					
			sss.move("head","front_face",False)
			self.object_recognition_stop()
			return 'failed'

		SAY("I see ")
		for recObject in resp.objects:
			SAY(recObject.name + ", ")

		for recObject in resp.objects:
			print "object name: " + recObject.name
		#	print "object position: " + recObject.pose.pose.position
			if userdata.object_name == recObject.name:
				userdata.grasp_position = recObject.pose.pose.position
				#SAY("I will now grasp the " + userdata.object_name)
				self.object_recognition_stop()
				return 'success'

		SAY("I could not find the " + userdata.object_name + ".")
		self.arm.set_named_target("folded")
		self.arm.go()
				
		sss.move("head","front_face",False)
		self.object_recognition_stop()
		return 'failed'
		
class find_any_known_object(smach.State):
	def __init__(self):	
		smach.State.__init__(self, outcomes=['success','failed'], output_keys=['grasp_position'])
		self.find_object_srv = rospy.ServiceProxy('/mcr_perception/object_recognition/get_object_list', mcr_perception_msgs.srv.GetObjects)
		self.object_recognition_start = rospy.ServiceProxy('/mcr_perception/object_recognition_height_based/start', std_srvs.srv.Empty)
		self.object_recognition_stop = rospy.ServiceProxy('/mcr_perception/object_recognition_height_based/stop', std_srvs.srv.Empty)
		self.arm = moveit_commander.MoveGroupCommander('arm')

	def execute(self, userdata):
		rospy.wait_for_service('/mcr_perception/object_recognition_height_based/start', 30)
		rospy.wait_for_service('/mcr_perception/object_recognition_height_based/stop', 30)
		rospy.wait_for_service('/mcr_perception/tabletop_segmentation/start', 30)
		rospy.wait_for_service('/mcr_perception/tabletop_segmentation/stop', 30)
		self.object_recognition_start()
		
		
		sss.move("head", "back_table", False)
		self.arm.set_named_target("look_at_table")
		self.arm.go()
		sss.move("torso", "extrem_back")

		rospy.sleep(5)

		rospy.wait_for_service('/mcr_perception/object_recognition/get_object_list', 30)
		for i in range(20): 
			print "calling /mcr_perception/object_recognition/get_object_list service"
			resp = self.find_object_srv()
			tempList = []
			for recObject in resp.objects:
				if recObject.name != "":
					tempList.append(recObject)
			
			resp.objects = tempList

			if (len(resp.objects) <= 0):
				print "no graspable objects found"
				rospy.sleep(1)
			else:
				break
				
		if (len(resp.objects) <= 0):
			SAY("I could not find any object.")
			self.arm.set_named_target("folded")
			self.arm.go()
					
			sss.move("head","front_face",False)
			self.object_recognition_stop()

			return 'failed'

		SAY("I see ")
		for recObject in resp.objects:
			SAY(recObject.name + ", ")

		object_to_grasp = resp.objects[0].name
		found_best_object = False
		### work just for Magdeburg 2012
		obj_priority_list = ['pringles','ketchup','can','glas','maggi','yoghurt','chocolate','book','cream','toy']
		
		#obj_priority_list = ['toy','cream','book','chocolate','yoghurt','maggi','glas','can','ketchup','pringles']
		
		for i in range(len(obj_priority_list)):
			if found_best_object:
				break
			
			for j in range(len(resp.objects)):
				if (resp.objects[j].name == obj_priority_list[i]):
					object_to_grasp = recObject
					found_best_object = True
					break
		
		'''
		for prio_obj in obj_priority_list:
			if found_best_object == True:
				break
				
			for recObject in resp.objects:
				if recObject == prio_obj:
					object_to_grasp = recObject
					found_best_object = True
					break
		'''
		####
		
		userdata.grasp_position = object_to_grasp.pose.pose.position
		SAY("I will now grasp the " + object_to_grasp.name)
		self.object_recognition_stop()

		return 'success'

class find_any_known_object_height_based(smach.State):
	def __init__(self):	
		smach.State.__init__(self, outcomes=['success','failed'], output_keys=['grasp_position'])
		self.find_object_srv = rospy.ServiceProxy('/mcr_perception/object_recognition/get_object_list', mcr_perception_msgs.srv.GetObjectList)
		self.object_recognition_start = rospy.ServiceProxy('/mcr_perception/object_recognition_height_based/start', std_srvs.srv.Empty)
		self.object_recognition_stop = rospy.ServiceProxy('/mcr_perception/object_recognition_height_based/stop', std_srvs.srv.Empty)
		self.arm = moveit_commander.MoveGroupCommander('arm')

	def execute(self, userdata):
		rospy.wait_for_service('/mcr_perception/object_recognition_height_based/start', 30)
		rospy.wait_for_service('/mcr_perception/object_recognition_height_based/stop', 30)
		self.object_recognition_start()
		
		sss.move("head", "back_table", False)
		self.arm.set_named_target("look_at_table")
		self.arm.go()
		sss.move("torso", "extrem_back")

		rospy.sleep(5)

		rospy.wait_for_service('/mcr_perception/object_recognition/get_object_list', 30)
		for i in range(20): 
			print "calling /mcr_perception/object_recognition/get_object_list service"
			resp = self.find_object_srv()

			if (len(resp.objects) <= 0):
				print "no graspable objects found"
				rospy.sleep(1)
			else:
				break
				
		if (len(resp.objects) <= 0):
			SAY("I could not find any object.")
			self.arm.set_named_target("folded")
			self.arm.go()

			sss.move("head","front_face",False)
			self.object_recognition_stop()
			return 'failed'

		SAY("I see ")
		for recObject in resp.objects:
			SAY(recObject.name + ", ")

		object_to_grasp = resp.objects.pop()
		userdata.grasp_position = object_to_grasp.pose.pose.position
		SAY("I will now grasp the " + object_to_grasp.name)
		self.object_recognition_stop()
		return 'success'


class categorize_objects(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['success','failed'], input_keys=['object_name'], output_keys=['object_name'])
		self.get_categorized_objects = rospy.ServiceProxy('/mcr_perception/object_categorization/get_categorized_objects', mcr_perception_msgs.srv.GetObjectList)
		self.object_categorization_start = rospy.ServiceProxy('/mcr_perception/object_categorization/start', std_srvs.srv.Empty)
		self.object_categorization_stop = rospy.ServiceProxy('/mcr_perception/object_categorization/stop', std_srvs.srv.Empty)
		self.object_recognition_height_based_start = rospy.ServiceProxy('/mcr_perception/object_recognition_height_based/start', std_srvs.srv.Empty)
		self.object_recognition_height_based_stop = rospy.ServiceProxy('/mcr_perception/object_recognition_height_based/stop', std_srvs.srv.Empty)
		self.arm = moveit_commander.MoveGroupCommander('arm')

	def execute(self, userdata):
		rospy.wait_for_service('/mcr_perception/object_recognition_height_based/start', 30)
		rospy.wait_for_service('/mcr_perception/object_recognition_height_based/stop', 30)
		rospy.wait_for_service('/mcr_perception/object_categorization/start', 30)
		rospy.wait_for_service('/mcr_perception/object_categorization/stop', 30)
		rospy.wait_for_service('/mcr_perception/object_categorization/get_categorized_objects', 5)
		
		self.get_categorized_objects()
		
		sss.move("head", "back_table", False)
		self.arm.set_named_target("look_at_table")
		self.arm.go()
		sss.move("torso", "extrem_back")
		rospy.sleep(5)

		self.object_recognition_height_based_start()
		self.object_categorization_start()
		
		exitLoop = False
		for i in range(50): 
			print "calling /mcr_perception/object_categorization/categorized_objects service"
			resp = self.get_categorized_objects()

			if (len(resp.objects) <= 0):
				print "no objects found"
				rospy.sleep(1)
			else:
				break
		
		
		if(len(resp.objects) > 0):
			SAY("I see")
			for object in resp.objects:
				SAY(object.name + ", ")
				
			#move everything to default positions
			sss.move("torso","home")
			self.arm.set_named_target("folded")
			self.arm.go()
			#self.object_categorization_stop()
			self.object_recognition_height_based_stop()

			sss.move("head","front_face",False)
			return 'success'	
		else:
			SAY("I could not find an object!")
		
		#move everything to default positions
		sss.move("torso","home")
		self.arm.set_named_target("folded")
		self.arm.go()
		#self.object_categorization_stop()
		self.object_recognition_height_based_stop()

		sss.move("head","front_face",False)
		return 'failed'
				

