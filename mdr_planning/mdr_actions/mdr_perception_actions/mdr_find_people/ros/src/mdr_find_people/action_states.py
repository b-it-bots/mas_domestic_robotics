import math
import rospy
import smach
import sympy
import numpy as np

import tf
import face_recognition
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import Point, Pose, PoseStamped
from mdr_find_people.msg import FindPeopleResult
from mas_perception_msgs.msg import Person, PersonList
from mas_perception_libs import ImageDetectionKey
from mas_perception_libs.visualization import crop_image
from mas_perception_libs.utils import cloud_msg_to_cv_image
from cv_bridge import CvBridge
from find_people import FindPeople


class FindPeopleState(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed'],
                             input_keys=['find_people_goal'],
                             output_keys=['find_people_result', 'error_message'])

        self._listener = tf.TransformListener()
        self.pointcloud_topic = rospy.get_param("~pointcloud_topic", '/rectified_points')


    def execute(self, userdata):
        rospy.loginfo('Executing state FIND_PEOPLE')

        # Get the pointcloud
        cloud_msg = rospy.wait_for_message(self.pointcloud_topic, PointCloud2)

        # Get positions of people
        predictions, bb2ds, poses = FindPeople.detect(cloud_msg)

        # Get people images
        cv_image = cloud_msg_to_cv_image(cloud_msg)
        bridge = CvBridge()
        images = []
        face_images = []
        for i, bb2d in enumerate(bb2ds):
            cropped_cv = crop_image(cv_image, bb2d)
            cropped_img_msg = bridge.cv2_to_imgmsg(cropped_cv, encoding="passthrough")
            
            rospy.loginfo('[find_people] Attempting to extract face of person {}'.format(i+1))
            cropped_face_img = self._extract_face_image(cropped_cv)
            if cropped_face_img is not None:
                cropped_face_img_msg = bridge.cv2_to_imgmsg(cropped_face_img,
                                                            encoding='passthrough')
            else:
                cropped_face_img_msg = Image()
            
            images.append(cropped_img_msg)
            face_images.append(cropped_face_img_msg)

        # Create the action result message
        pl = []
        for i, _ in enumerate(predictions):
            p = Person()
            p.identity = str(i)
            p.probability = predictions[i][ImageDetectionKey.CONF]

            map_pose = self._listener.transformPose('/map', poses[i])
            p.pose = map_pose
            p.body_image = images[i]
            p.face.image = face_images[i]

            pl.append(p)

        # Package that actual PersonList message
        person_list = PersonList()
        person_list.persons = pl
        result = FindPeopleResult()
        result.person_list = person_list

        userdata['find_people_result'] = result
        return 'succeeded'

    def _extract_face_image(self, image_array):
        try:
            top, right, bottom, left = face_recognition.face_locations(image_array)[0]
            rospy.loginfo('[find_people] Successfully extracted face from person image.')
            return image_array[top:bottom, left:right]
        except IndexError:
            rospy.logwarn('[find_people] Failed to extract face from person image!')
            return None
