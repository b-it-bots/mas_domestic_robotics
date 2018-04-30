#!/usr/bin/env python

import rospy
import actionlib
import simple_script_server
import geometry_msgs.msg
import tf
import math
import numpy
from numpy import linalg as la
import moveit_commander
from moveit_msgs.srv import GetStateValidityResponse, GetStateValidity, GetStateValidityRequest
from moveit_msgs.msg import RobotState

import mdr_behaviors_msgs.srv
import kinematics
import move_arm


class PickupPlaceServer:

	def __init__(self):
		self.group_name_arm = 'arm'
		self.group_name_gripper = 'gripper'
		self.kinematics = kinematics.Kinematics(self.group_name_arm)
		self.move_arm = move_arm.MoveArm(self.group_name_arm)
		self.robot = moveit_commander.RobotCommander()
		self.commander = moveit_commander.MoveGroupCommander(self.group_name_arm)
		self.sv_srv = rospy.ServiceProxy("/check_state_validity", GetStateValidity)
		rospy.loginfo("Connecting to State Validity service")
		self.sv_srv.wait_for_service()
		self.group = group = moveit_commander.MoveGroupCommander(self.group_name_arm)
		# distance from object center to pre-grasp
		self.pre_grasp_starting_distance = rospy.get_param('~pre_grasp_distance')
		# distance from object to grasp
		self.grasp_distance = rospy.get_param('~grasp_distance')
		# height of the post-grasp over the object center
		self.post_grasp_height = rospy.get_param('~post_grasp_height')
		# at which angle of approach to start searching for a solution
		self.end_angle =  math.radians(65);
		self.service_type_pickup = "pickup"
		self.service_type_place = "place"
                self.pub_selected_pose = rospy.Publisher('selected_pose', geometry_msgs.msg.PoseStamped, queue_size=1);
                self.compensation_factor = 650;

		rospy.loginfo('Pre-grasp distance: %f', self.pre_grasp_starting_distance)
		rospy.loginfo('Grasp distance: %f', self.grasp_distance)
		rospy.loginfo('Post-grasp height: %f', self.post_grasp_height)

		self.sss = simple_script_server.simple_script_server()

		# service servers
		self.pickup_server = rospy.Service(self.service_type_pickup, mdr_behaviors_msgs.srv.Pickup, self.pickup)
		rospy.loginfo('"pickup" service advertised')

		# service servers
		self.place_server = rospy.Service(self.service_type_place, mdr_behaviors_msgs.srv.Place, self.place)
		rospy.loginfo('"place" service advertised')

	def is_state_valid(self, joint_positions):
		groups = [self.group_name_arm]
		result = True
		for group in groups:
			gsvr = GetStateValidityRequest()
			robot_state = RobotState()
			robot_state.joint_state.name = self.commander.get_joints()
			robot_state.joint_state.position = joint_positions
			gsvr.robot_state = robot_state
			#Check arm collisions
			gsvr.group_name = group
			result_ = self.sv_srv.call(gsvr)
			#Check gripper collisions
			rospy.loginfo("Service result for group %s: %s", group, result_.valid)
			result = result and result_.valid
		rospy.loginfo("Final result: %s", result)
		return result

	def pickup(self, req):
		print "*****Pick up service******"
		return self.plan_and_move(self.service_type_pickup, req)

	def place(self, req):
                print "******Place service******"
		return self.plan_and_move(self.service_type_place, req)

	def plan_and_move(self, service_type, req):
		'''
		Perform the place/pick task

		:param req: mdr_behaviors_msgs.PlaceRequest
		req.position: geometry_msgs.msg.PointStamped
		'''
		rospy.logdebug('Start planning trajectory')

		req.position.point.y -= 0.0

		# pre-calculate the trajectory to the position
		traj = self.plan_trajectory(req.position)

		#self.calculate_distance_between_trajectories(traj)

		if (not traj):
			rospy.loginfo('No trajectory found')
			if service_type == self.service_type_place:
				res = mdr_behaviors_msgs.srv.PlaceResponse()
			else:
				res = mdr_behaviors_msgs.srv.PickupResponse()
			res.success = False
			return res

		rospy.loginfo('Found a trajectory')

		#Move arm
		#Open gripper
		if service_type == self.service_type_pickup:
			sdh_handle = self.sss.init('gripper', blocking = True)
			sdh_handle = self.sss.recover('gripper', blocking = True)
			sdh_handle = self.sss.move('gripper', 'cylopen', blocking = False)

		rospy.loginfo('Moving to pre-location')
		# move the arm along the calculated trajectory to the pregrasp
		self.move_arm.move(traj[0:len(traj) - 3])

		if service_type == self.service_type_pickup:
			rospy.loginfo('Picking up object')        
			sdh_handle = self.sss.init('gripper', blocking = True)
			sdh_handle = self.sss.recover('gripper', blocking = True)
			sdh_handle = self.sss.move('gripper', 'cylclosed', blocking = True)
		else:
			rospy.loginfo('Placing object')   
			sdh_handle = self.sss.init('gripper', blocking = True)
			sdh_handle = self.sss.recover('gripper', blocking = True)
			sdh_handle = self.sss.move('gripper', 'cylopen', blocking = True)

		# move to post-grasp
		rospy.loginfo('Moving to post-location')
		self.move_arm.move(traj[len(traj) - 3:len(traj)])
		#self.commander(traj[len(traj) - 3:len(traj)])

		#return response
		if service_type == self.service_type_pickup:       
			res = mdr_behaviors_msgs.srv.PickupResponse()
			rospy.loginfo('Picked up the object successfully')
		else:
			res = mdr_behaviors_msgs.srv.PlaceResponse()
			rospy.loginfo('Placed the object successfully')		
		res.success = True

		return res			

	def plan_trajectory(self, position):
		'''
		Create a trajectory from the current position to the grasp position. An
		intermediate pre-grasp in front of the object is inserted between
		current and goal position.

		:param position: geometry_msgs.msg.PointStamped
		:return: None or ([Float[7], ..., Float[7]]
		'''

		# extract the the pre-grasp pose from the parameter server
		pregrasp_param = rospy.get_param('/script_server/arm/home')
		#prepregrasp_param = rospy.get_param('/script_server/arm/prepregrasp')
		#prepregrasp = prepregrasp_param[0]
		init_configuration = pregrasp_param[0]

		#pregrasp back_front
		prepregrasp = [-1.478894, -1.319538, 0.235023, 2.076411, -1.45696, 1.652773, 0.500008]
		prepregrasp_plan = [-1.89452, -0.651637, 0.251916, 1.159974, -1.46001, 1.653793, 2.000047]

		angle_increment = 0.02
                angle_factor = 1
		angle =  angle_factor * angle_increment
		increment = False
		if(position.point.y < 0.130):
			increment = True
			angle = -angle_factor * angle_increment

                #Grasping sideways
                if(position.point.y < -0.50):
                        angle = math.radians(45)
                	self.end_angle = math.radians(145)
			
		while (abs(angle) < self.end_angle):

			# first we try to find a solution that is oriented with the hand's
			# pre-grasp pose later grasps try to apprach angles as close from 0 as possible.
			if (increment):
				angle = angle + angle_increment
			else:
				angle = angle - angle_increment

			rospy.loginfo('Planning grasp with approach angle: %f', math.degrees(angle))


			pre_grasp_distance = self.pre_grasp_starting_distance
			dist_decrement = 0.03
			traj_pregrasp_top_back_grasp = list()

			while (pre_grasp_distance  > 0.3555554):
                            if self.can_calculate(position, pre_grasp_distance, angle, self.post_grasp_height, True):
				traj_pregrasp_top_back_grasp = self.calculate_trajectory_abs(position,pre_grasp_distance , angle, self.post_grasp_height,  prepregrasp_plan, True)
				if(traj_pregrasp_top_back_grasp):
				    rospy.loginfo('Found pregrasp_distance %f', pre_grasp_distance)
				    break
                            else:
                                rospy.loginfo("Possible selfcollition detected")
			    pre_grasp_distance = pre_grasp_distance - dist_decrement
			
			if (not traj_pregrasp_top_back_grasp):
			    continue
			elif not self.is_state_valid(traj_pregrasp_top_back_grasp):
				rospy.loginfo('Found pre-top-back-location but collides with an object in the environment')
				continue
			rospy.loginfo('Found pre-top-back-location')
			
			# determine the trajectory to the post-grasp pose
			traj_pregrasp_top_grasp = self.calculate_trajectory_abs(position, self.grasp_distance, angle, self.post_grasp_height, traj_pregrasp_top_back_grasp)
			if (not traj_pregrasp_top_grasp ):
			    continue
			elif not self.is_state_valid(traj_pregrasp_top_grasp):
				rospy.loginfo('Found pre-top-location but collides with an object in the environment')
				continue
			rospy.loginfo('Found pre-top-location')			
			
			#determine the trajectory to the grasp pose
			traj_grasp = self.calculate_trajectory_abs(position, self.grasp_distance, angle, 0.02, traj_pregrasp_top_grasp)
			if (not traj_grasp ):
			    continue
			elif not self.is_state_valid(traj_grasp):
				rospy.loginfo('Found grasp/place but collides with an object in the environment')
				continue
			rospy.loginfo('Found grasp/place')
					
			rospy.loginfo('Checking trajectories pregrasp-back-top/ pregrasp-top')
			if not self.check_trajectories(traj_pregrasp_top_back_grasp, traj_pregrasp_top_grasp):
			    continue
			
			rospy.loginfo('Checking trajectories pregrasp-top/ graps')			
			if not self.check_trajectories(traj_pregrasp_top_grasp, traj_grasp):
			    continue
			
			self.is_state_valid(traj_grasp)
			#if all trajectories have been determined we return the result
			return [list(prepregrasp), list(traj_pregrasp_top_back_grasp), list(traj_grasp), list(traj_pregrasp_top_grasp), list(traj_pregrasp_top_back_grasp), list(prepregrasp)]

		return None
	
	def check_trajectories(self, trajectory1, trajectory2):
		if (numpy.absolute(numpy.degrees(trajectory1[0]) - numpy.degrees(trajectory2[0]))) > 100:
			rospy.loginfo('***********Joint one not valid***********')
			return False
		if (numpy.absolute(numpy.degrees(trajectory1[1]) - numpy.degrees(trajectory2[1]))) > 100:
			rospy.loginfo('***********Joint two not valid***********')
			return False
		if (numpy.absolute(numpy.degrees(trajectory1[2]) - numpy.degrees(trajectory2[2]))) > 100:
			rospy.loginfo('***********Joint three not valid***********')
			return False
		if (numpy.absolute(numpy.degrees(trajectory1[3]) - numpy.degrees(trajectory2[3]))) > 100:
			rospy.loginfo('***********Joint four not valid***********')
			return False
		if (numpy.absolute(numpy.degrees(trajectory1[4]) - numpy.degrees(trajectory2[4]))) > 100:
			rospy.loginfo('***********Joint five not valid***********')
			return False
		if (numpy.absolute(numpy.degrees(trajectory1[5]) - numpy.degrees(trajectory2[5]))) > 75:
			rospy.loginfo('***********Joint six not valid***********')
			return False
		if (numpy.absolute(numpy.degrees(trajectory1[6]) - numpy.degrees(trajectory2[6]))) > 50:
			rospy.loginfo('***********Joint six not valid***********')
			return False			
		return True


	def calculate_trajectory_abs(self, position, distance, angle, height, configuration, compensate = False):
		'''
		Based on the goal position, the distance and angle from the goal
		position and a given configuration determine the joint angles to reach
		the distance in front of the provided position.

		:param position: geometry_msgs.msg.PointStamped
		:param distance: Float
		:param angle: Float
		:param height: Float
		:param configuration: Float[7]
		:return: None or Float[7]
		'''
		pose = geometry_msgs.msg.PoseStamped()
		pose = self.calculate_pose_abs(position, distance, angle, height, compensate)

		# determine the inverse kinematics solution
		return self.kinematics.inverse_kinematics(pose, configuration)
	


	def calculate_pose_abs(self, position, distance, angle, height, compensate = False):
		'''
		Determine a position that is the given distance in front of the provided
		position. The angle is given in the object frame and describes the angle
		of approach to the provided position.

		:param position: geometry_msgs.msg.PointStamped
		:param distance: Float
		:param angle: Float
		:param height: Float
		:return: geometry_msgs.msg.PoseStamped
		'''

		pose = geometry_msgs.msg.PoseStamped()
		pose.header = position.header
                
                distance_ = distance
                if compensate:
                    distance_ = distance + (abs(numpy.degrees(angle)) / self.compensation_factor )
		
                deltaX = distance_ * math.cos(angle)
		deltaY = distance_ * math.sin(angle)
		roll = math.pi
		pitch = math.pi / 2.0
		yaw = angle

		quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

		pose.pose.position.x = position.point.x + deltaX
		pose.pose.position.y = position.point.y + deltaY
		pose.pose.position.z = position.point.z + height
		pose.pose.orientation.x = quaternion[0]
		pose.pose.orientation.y = quaternion[1]
		pose.pose.orientation.z = quaternion[2]
		pose.pose.orientation.w = quaternion[3]
		
                self.pub_selected_pose.publish(pose)
		return pose

        def can_calculate(self, position, distance, angle, height, compensate = False):
		'''
		Determine a position that is the given distance in front of the provided
		position. The angle is given in the object frame and describes the angle
		of approach to the provided position.

		:param position: geometry_msgs.msg.PointStamped
		:param distance: Float
		:param angle: Float
		:param height: Float
		:return: geometry_msgs.msg.PoseStamped
		'''

		pose = geometry_msgs.msg.PoseStamped()
		pose.header = position.header

                distance_ = distance
                if compensate:
                    distance_ = distance + (abs(numpy.degrees(angle)) / self.compensation_factor )
                
                rospy.loginfo("Distance: %f: ", distance_)
		deltaX = distance_ * math.cos(angle)
		deltaY = distance_ * math.sin(angle)
                rospy.loginfo("x: %f", position.point.x + deltaX)
                if (position.point.z + height < 0.90):
                    rospy.loginfo("Height might couase a collition with the torse %f", position.point.z + height)
                    if(position.point.x + deltaX > -0.20):
                        return False
                elif position.point.x + deltaX > -0.10:
                    return False
                return True


def main():
	rospy.init_node('pickup_and_place')

	server = PickupPlaceServer()

	rospy.spin()
