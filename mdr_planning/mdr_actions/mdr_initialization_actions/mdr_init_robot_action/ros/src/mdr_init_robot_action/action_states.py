#!/usr/bin/python

import rospy
import smach
import smach_ros
import actionlib
import moveit_commander
from std_srvs.srv import Empty

import simple_script_server
from mdr_init_robot_action.msg import InitRobotGoal, InitRobotFeedback, InitRobotResult

class SetupInitRobot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             input_keys=['init_robot_goal'],
                             output_keys=['init_robot_feedback', 'init_robot_result'])

    def execute(self, userdata):
        feedback = InitRobotFeedback()
        feedback.current_state = 'INIT_ROBOT'
        feedback.message = '[init_robot] initialising the robot'
        userdata.init_robot_feedback = feedback

        return 'succeeded'

class InitRobot(smach.State):
    def __init__(self, timeout=120.0, arm_name='arm',
                 arm_recover_srv_name='/arm_controller/lwr_node/recover'):
        smach.State.__init__(self, input_keys=['init_robot_goal'],
                             output_keys=['initialised_components'],
                             outcomes=['succeeded', 'failed'])
        self.timeout = timeout
        self.sss = simple_script_server.simple_script_server()

        self.arm = moveit_commander.MoveGroupCommander(arm_name)
        self.arm_recover_srv_name = arm_recover_srv_name
        self.arm_recover_client = rospy.ServiceProxy(self.arm_recover_srv_name, Empty)

    def execute(self, userdata):
        initialised_components = list()
        for component in userdata.init_robot_goal.components:
            # the arm can be initialised by making a controller service call
            if component == InitRobotGoal.ARM and component not in initialised_components:
                try:
                    rospy.wait_for_service(self.arm_recover_srv_name, 5)
                    self.arm_recover_client()
                    initialised_components.append(InitRobotGoal.ARM)
                except (rospy.ROSException, rospy.ServiceException), e:
                    rospy.logerr('Service call to {0} failed: {1}'.format(self.arm_recover_srv_name, e))
            # we can initialise the other components by making calls to the simple script server
            elif (component == InitRobotGoal.BASE or component == InitRobotGoal.TRAY or
            component == InitRobotGoal.HEAD or component == InitRobotGoal.GRIPPER or
            component == InitRobotGoal.TORSO) and component not in initialised_components:
                # initialing and recovering the component
                self.sss.init(component)
                self.sss.recover(component)

                # moving the component to its default state
                component_handle = None
                if component == InitRobotGoal.TRAY:
                    component_handle = self.sss.move(InitRobotGoal.TRAY, 'down', False)
                elif component == InitRobotGoal.HEAD:
                    component_handle = self.sss.move(InitRobotGoal.HEAD, 'front', False)
                elif component == InitRobotGoal.GRIPPER:
                    component_handle = self.sss.move(InitRobotGoal.GRIPPER, 'cylclosed', False)
                elif component == InitRobotGoal.TORSO:
                    component_handle = self.sss.move(InitRobotGoal.TORSO, 'home', False)

                # checking if the component was initialised successfully
                error_code = 0
                if component_handle is not None:
                    error_code = component_handle.get_error_code()

                if error_code == 0:
                    rospy.loginfo('{0} initialised successfully'.format(component))
                    initialised_components.append(component)
                else:
                    rospy.logerr('{0} failed to initialise'.format(component))
            # we ignore unknown components
            else:
                rospy.loginfo('[init_robot] Unknown component {0} specified; skipping component initialisation'.format(component))

        userdata.initialised_components = initialised_components
        return 'succeeded'

class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['init_robot_goal', 'initialised_components'],
                             output_keys=['init_robot_feedback', 'init_robot_result'])
        self.result = result

    def execute(self, userdata):
        result = InitRobotResult()
        for component in userdata.init_robot_goal.components:
            if component in userdata.initialised_components:
                result.success.append(True)
            else:
                result.success.append(False)
        userdata.init_robot_result = result
        return 'succeeded'
