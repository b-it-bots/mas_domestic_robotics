import yaml

import rospy
import tf
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3

from mas_perception_msgs.msg import Object, ObjectList
from mdr_manipulation_msgs.srv import UpdatePlanningScene, UpdatePlanningSceneRequest

from mas_tools.ros_utils import get_package_path

from mas_execution_manager.scenario_state_base import ScenarioStateBase

def get_environment_objects(planning_scene_map_file):
    object_list = ObjectList()

    scene_file_path = get_package_path('mdr_wrs_tidy_up', 'config', planning_scene_map_file)
    scene_data = None
    with open(scene_file_path, 'r') as planning_scene_map:
        scene_data = yaml.safe_load(planning_scene_map)

    frame_id = scene_data['frame_id']
    for obj_data in scene_data['objects']:
        obj = Object()
        obj.name = obj_data['name']
        obj.pose.header.frame_id = frame_id

        euler_orientation = obj_data['orientation']
        quaternion = tf.transformations.quaternion_from_euler(*euler_orientation)

        obj.pose.pose = Pose(Point(*obj_data['position']), Quaternion(*quaternion))
        obj.dimensions.header.frame_id = frame_id
        obj.dimensions.vector = Vector3(*obj_data['dimensions'])
        obj.bounding_box.center = Point(*obj_data['position'])
        obj.bounding_box.dimensions = Vector3(*obj_data['dimensions'])
        object_list.objects.append(obj)
    return object_list


class InitialiseScenario(ScenarioStateBase):
    floor_objects_cleared = None
    table_objects_cleared = None
    object_location = ''
    planning_scene_update_service_name = ''
    planning_scene_update_proxy = None
    planning_scene_map_file = None

    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'initialise_scenario',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded'],
                                   output_keys=['floor_objects_cleared',
                                                'table_objects_cleared',
                                                'object_location',
                                                'environment_objects'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'initialise_scenario')
        self.floor_objects_cleared = kwargs.get('floor_objects_cleared', None)
        self.table_objects_cleared = kwargs.get('table_objects_cleared', None)
        self.object_location = kwargs.get('object_location', 'floor')
        self.planning_scene_map_file = kwargs.get('planning_scene_map_file', '')
        self.planning_scene_update_service_name = kwargs.get('planning_scene_update_service_name',
                                                             '/move_arm_action/update_planning_scene')
        self.__init_ros_components()

    def execute(self, userdata):
        userdata.floor_objects_cleared = self.floor_objects_cleared
        userdata.table_objects_cleared = self.table_objects_cleared
        userdata.object_location = self.object_location

        if not self.planning_scene_map_file:
            rospy.loginfo('[%s] Planning scene map file not specified; not initialising KB and scene', self.state_name)
            return 'succeeded'

        environment_objects = get_environment_objects(self.planning_scene_map_file)

        # initialising the knowledge base
        rospy.loginfo('[%s] Initialing knowledge base', self.state_name)
        userdata.environment_objects = self.__init_kb(environment_objects, userdata)
        rospy.loginfo('[%s] Knowledge base initialised', self.state_name)

        # initialising the MoveIt! planning scene
        update_planning_scene_req = UpdatePlanningSceneRequest()
        update_planning_scene_req.operation = UpdatePlanningSceneRequest.ADD
        update_planning_scene_req.objects = environment_objects

        rospy.loginfo('[%s] Initialising planning scene', self.state_name)
        response = self.planning_scene_update_proxy(update_planning_scene_req)
        if response is not None:
            if response.success:
                rospy.loginfo('[%s] Successfully updated the planning scene', self.state_name)
            else:
                rospy.logerr('[%s] Failed to update the planning scene', self.state_name)
        else:
            rospy.logerr('[%s] Response not received', self.state_name)

        return 'succeeded'

    def __init_kb(self, object_list, userdata):
        kb_objects = dict()
        try:
            for obj in object_list.objects:
                # we don't add walls and table legs to the knowledge base
                if obj.name.find('wall') != -1 or obj.name.find('leg') != -1:
                    continue
                kb_objects[obj.name] = obj
        except Exception as exc:
            rospy.logerr('[%s] Error while initialising the knowledge base', self.state_name)
            rospy.logerr('[%s] %s', self.state_name, str(exc))
        return kb_objects

    def __init_ros_components(self):
        rospy.loginfo('[%s] Creating a service proxy for %s',
                      self.state_name, self.planning_scene_update_service_name)
        rospy.wait_for_service(self.planning_scene_update_service_name)
        self.planning_scene_update_proxy = rospy.ServiceProxy(self.planning_scene_update_service_name,
                                                              UpdatePlanningScene)
        rospy.loginfo('[%s] Service proxy for %s created',
                      self.state_name, self.planning_scene_update_service_name)
