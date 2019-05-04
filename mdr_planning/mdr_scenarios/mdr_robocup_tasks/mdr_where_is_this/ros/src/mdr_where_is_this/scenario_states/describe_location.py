import rospy
from geometry_msgs.msg import PoseStamped
from mas_execution_manager.scenario_state_base import ScenarioStateBase
from topological_map_ros.srv import TopologicalPath, TopologicalPathRequest, \
                                    TopologicalPosition, TopologicalPositionRequest

class DescribeLocation(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'describe_location',
                                   save_sm_state=save_sm_state,
                                   input_keys=['target_entity'],
                                   outcomes=['succeeded', 'failed',
                                             'failed_after_retrying'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'describe_location')
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.pose_timeout = kwargs.get('pose_timeout', 5.)
        self.pose_topic_name = kwargs.get('pose_topic_name', '/amcl_pose')
        self.retry_count = 0

        self.current_pose = None
        self.current_top_pos = None
        self.pose_sub = rospy.Subscriber('/amcl_pose', PoseStamped, self.get_pose)

        rospy.loginfo('[describe_location] Waiting for topological_position server')
        self.topological_position_client = rospy.ServiceProxy('topological_position',
                                                              TopologicalPosition)
        rospy.loginfo('[describe_location] topological_position server up')

        rospy.loginfo('[describe_location] Waiting for topological_path_plan server')
        self.topological_path_client = rospy.ServiceProxy('topological_path_plan',
                                                          TopologicalPath)
        rospy.loginfo('[describe_location] topological_path_plan server up')

    def execute(self, userdata):
        if self.save_sm_state:
            self.save_current_state()

        try:
            self.current_pose = rospy.wait_for_message(self.pose_topic_name,
                                                       PoseStamped,
                                                       self.pose_timeout)
        except rospy.ROSException as exc:
            rospy.logerr(str(exc))
            self.say('I unfortunately don\'t know where I am')
            return 'failed'

        top_pos_request = TopologicalPositionRequest()
        top_pos_request.current_pose = self.current_pose
        try:
            top_pos_result = self.topological_position_client(top_pos_request)
            self.current_top_pos = top_pos_result.area
        except rospy.ServiceException as exc:
            self.say('I\'m still unsure where I am')
            rospy.logerr(str(exc))
            return 'failed'

        obj_name = userdata.target_entity['value']
        location = ''
        directions = None
        if userdata.target_entity['type'] == 'location':
            location = userdata.target_entity['value']
        elif userdata.target_entity['type'] == 'object':
            location = self.ontology_interface.get_obj_location(obj_name)
            if not location:
                self.say('I unfortunately don\'t know where the {0} is'.format(obj_name))
                return 'failed'

        top_path_request = TopologicalPathRequest()
        top_path_request.source = self.current_top_pos
        top_path_request.goal = location
        try:
            top_path_result = self.topological_path_client(top_path_request)
            directions = top_path_result.directions
        except rospy.ServiceException as exc:
            self.say('I don\'t know how to get to the {0}'.format(location))
            rospy.logerr(str(exc))
            return 'failed'

        if userdata.target_entity['type'] == 'location':
            self.say('You can reach the {0} as follows'.format(location))
            for path_description in directions:
                self.say(path_description)
        elif userdata.target_entity['type'] == 'object':
            self.say('The {0} is in the {1}'.format(obj_name, location))
            self.say('You can reach there as follows')

            for path_description in directions:
                self.say(path_description)

            objects_next_to = self.ontology_interface.get_objects_next_to(obj_name)
            if objects_next_to:
                self.say('The {0} is next to {1}'.format(obj_name,
                                                         self.format_obj_next_to_list(objects_next_to)))
        return 'succeeded'

    def format_obj_next_to_list(self, obj_list):
        obj_list_str = ''
        for obj in range(len(obj_list)-2):
            obj_list_str += obj + ', '
        obj_list_str += ' and the ' + obj_list[-1]
        return obj_list_str

    def get_pose(self, pose_msg):
        self.current_pose = pose_msg.pose
