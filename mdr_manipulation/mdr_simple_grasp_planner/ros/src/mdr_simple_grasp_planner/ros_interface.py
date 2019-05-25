import rospy
import moveit_msgs.msg
import std_msgs.msg
import grasp_planner

class GraspPlannerRosInterface:

    _STATE_CREATED = 0
    _STATE_IDLE = 1
    _STATE_RESET = 2
    _STATE_TRIGGERED = 3

    def __init__(self):
        self.planner = grasp_planner.GraspPlanner()
        self.grasp_publisher = rospy.Publisher('~grasp', moveit_msgs.msg.Grasp)
        self.event_subscriber = rospy.Subscriber('~event_in',
                std_msgs.msg.String, self.event_in, queue_size = 1)
        self.event_publisher = rospy.Publisher('~event_out', std_msgs.msg.String)
        self.state = self._STATE_CREATED
        self.next_grasp = 0
        self.grasps = None

        rospy.loginfo('Grasp planner running')


    def event_in(self, msg):
        '''
        Handle an incoming event.

        :param msg: The event type. Valid events are:
        e_reset: Reset the planner and re-plan.
        e_trigger: Trigger the planner to send out the next grasp.
        :type msg: std_msgs.msg.String
        '''
        if (msg.data == 'e_reset'):
            self.state = self._STATE_RESET
        elif (msg.data == 'e_trigger'):
            if (self.state != self._STATE_CREATED):
                self.state = self._STATE_TRIGGERED
            else:
                rospy.logerr('Grasp planner is not initialized yet')
        else:
            rospy.logerr('Grasp planner received an invalid event')


    def reset(self):
        '''
        Reset the grasp planner and re-plan.
        '''
        rospy.logdebug('Planning new grasps.')
        self.grasps = self.planner.plan()
        self.next_grasp = 0
        self.event_publisher.publish('e_done')


    def handle_request(self):
        '''
        Handle a request by sending out a grasp.
        '''
        self.grasp_publisher.publish(self.grasps[self.next_grasp])
        self.next_grasp += 1

        # if all grasps have been processed inform the external components
        if (self.next_grasp == len(self.grasps)):
            self.next_grasp = 0
            self.event_publisher.publish('e_done')


    def step(self):
        '''
        Run one step of the state machine and execute the functionality
        associated with each state.
        '''
        if (self.state == self._STATE_CREATED):
            self.state = self._STATE_CREATED
        elif (self.state == self._STATE_IDLE):
            self.state = self._STATE_IDLE
        elif (self.state == self._STATE_RESET):
            self.reset()
            self.state = self._STATE_IDLE
        elif (self.state == self._STATE_TRIGGERED):
            self.handle_request()
            self.state = self._STATE_IDLE


def main():
    rospy.init_node('grasp_planner')
    planner = GraspPlannerRosInterface()    
    while (not rospy.is_shutdown()):
        planner.step()
        rospy.sleep(0.2)
