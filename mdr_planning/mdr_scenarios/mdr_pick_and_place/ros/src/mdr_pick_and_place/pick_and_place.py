#!/usr/bin/python

import rospy
import smach
import smach_ros

import pick_and_place_states as pps
import mdr_common_states.common_states as cs
import geometry_msgs.msg
	
def main():
	rospy.init_node('pick_and_place')

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['overall_success', 'overall_failed'])
	
	# base poses:
	# - home
	# - table_1
	# - table_2
	
	sm.userdata.pose = 'home'
	
	sm.userdata.action_list = []
	sm.userdata.action_object = None
	sm.userdata.grasp_pose = None
	
	sm.userdata.place_pose = geometry_msgs.msg.PointStamped()
	sm.userdata.place_pose.header.stamp = rospy.Time.now()
	sm.userdata.place_pose.header.frame_id = 'base_link'
	sm.userdata.place_pose.point.x = -0.8
	sm.userdata.place_pose.point.y =  0.0
	sm.userdata.place_pose.point.z =  0.8
	
	sm.userdata.already_executed_actions = []
	sm.userdata.current_action = None
	
	sm.userdata.room_poses = []
	
	# Open the container
	with sm:
		
		# init base
		smach.StateMachine.add('init_base', cs.init_base(),
				transitions={'success':'init_torso',
							'failed':'overall_failed'})
		
		# init torso
		smach.StateMachine.add('init_torso', cs.init_torso(),
				transitions={'success':'init_manipulator',
							'failed':'overall_failed'})
	
		# init manipulator
		smach.StateMachine.add('init_manipulator', pps.init_manipulator(),
				transitions={'success':'wait_for_start'})
	
		# wait for start
		smach.StateMachine.add('wait_for_start', cs.wait_for_start(),
				transitions={'success':'goto_home',
							'pending':'wait_for_start'})
		
		
		
		smach.StateMachine.add('goto_home', cs.approach_pose('home'),
				transitions={'success':'goto_table_1',
							'failed':'goto_home'})
		
		
		
		########################################################################
		# Pick
		########################################################################
		
		smach.StateMachine.add('goto_table_1', cs.approach_pose('table_1'),
				transitions={'success':'identify_object',
							'failed':'goto_table_1'})
		
		#grasp
		smach.StateMachine.add('identify_object', pps.find_any_known_object_height_based(),
				transitions={'success':'pickup_object', # grasp if found
							'failed':'goto_table_1'}, # otherwise search again
				remapping={'grasp_position':'grasp_pose',
						'object_name':'action_object'})
			
		# grasp Item
		smach.StateMachine.add('pickup_object', cs.pickup_object(),
				transitions={'success':'goto_table_2'},
				remapping={'grasp_position':'grasp_pose'})
		
		
		
		########################################################################
		# Place
		########################################################################
		
		smach.StateMachine.add('goto_table_2', cs.approach_pose('table_2'),
				transitions={'success':'place_object',
							'failed':'goto_table_2'})
		
		smach.StateMachine.add('place_object', cs.place_object(),
				transitions={'success':'goto_home'},
				remapping={'place_position':'place_pose'})


	# Create and start the introspection server
	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()

	# Execute the state machine
	#rospy.sleep(5)
	outcome = sm.execute()
	
	# Wait for ctrl-c to stop the application
	rospy.spin()
	sis.stop()



if __name__ == '__main__':
	main()
