import math
import rospy
import smach
from sympy import Polygon, Point

import tf
from mdr_find_people.msg import FindPeopleResult
from mcr_perception_msgs.msg import Person, PersonList
from mas_perception_libs import ImageDetectionKey
from find_people import FindPeople


class FindPeopleState(smach.State):

    @staticmethod
    def pose_subtract(pose, distance):
        """Subtracts the given distance from the pose,
           i.e. move by the given distance towards the origin/frame
        """
        point = pose.pose.position
        plen = math.sqrt(point.x ** 2 + point.y ** 2 + point.z ** 2)
        new_len = max(0, plen - distance)
        factor = new_len / plen

        new_point = Point(x=factor*point.x, y=factor*point.y, z=factor*point.z)
        result_pose = PoseStamped(header=pose.header, pose=Pose(position=new_point, orientation=pose.pose.orientation))
        return result_pose


    @staticmethod
    def is_inside_arena(pose):
        p1, p2, p3, p4 = map(Point, [(-0.9909883, -4.218833), (-1.92709, 0.9022037), (-7.009388, -1.916794), (-4.107592, -7.078834)])
        living_room = Polygon(p1,p2,p3,p4)
        person_pose = Point(pose.pose.position.x, pose.pose.position.y) # Inspection test pose
        return living_room.encloses_point(person_pose)


    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed'],
                             input_keys=['find_people_goal'],
                             output_keys=['find_people_result', 'error_message'])

	self._listener = tf.TransformListener()


    def execute(self, userdata):
        rospy.loginfo('Executing state FIND_PEOPLE')
        
        # Get positions of people
        predictions, bb2ds, poses = FindPeople.single_shot_detection()

        # Filter detections for people inside the arena
        for i in range(len(predictions)):
            if not FindPeopleState.is_inside_arena(poses[i]):
                del predictions[i]
                del bb2ds[i]
                del poses[i]

        # Create the action result message
        pl = []
        for i in range(len(predictions)):
            p = Person()
            p.id = i
            p.probability = predictions[i][ImageDetectionKey.CONF]

            map_pose = self._listener.transformPose('/map', poses[i])
            p.pose = map_pose

            # Calculate a safe pose for approching the person
            # 1 meter towards the robot from the actual position
            safe_pose = FindPeopleState.pose_subtract(poses[i], 1)
            p.safe_pose = safe_pose

            pl.append(p)

        # Package that actual PersonList message
        person_list = PersonList()
        person_list.persons = pl
        result = FindPeopleResult()
        result.person_list = person_list

        userdata['find_people_result'] = result
        return 'succeeded'
