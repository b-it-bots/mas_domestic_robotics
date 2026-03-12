#!/usr/bin/env python3
"""
VLM SMACH States

States that call the /vlm/query service (running on slave laptop)
with an image grabbed from the robot's head camera.

Available states:
    CheckSeatEmpty      -> outcomes: left | right | both | none | failed
    CheckDoorState      -> outcomes: open | closed | failed
    GetShelfPlacement   -> outcomes: top | middle | bottom | failed
"""

import base64
import rospy
import smach
import cv2
import numpy as np

try:
    from sensor_msgs.msg import Image, CompressedImage
    from cv_bridge import CvBridge
    BRIDGE = CvBridge()
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False

try:
    from hsr_task_sm.srv import VLMQuery, VLMQueryRequest
    VLM_SRV_AVAILABLE = True
except ImportError:
    VLM_SRV_AVAILABLE = False

# Default camera topic on HSR
DEFAULT_CAMERA_TOPIC = '/hsrb/head_rgbd_sensor/rgb/image_rect_color'
VLM_SERVICE = '/vlm/query'
SERVICE_TIMEOUT = 10.0


def _grab_image_base64(camera_topic=DEFAULT_CAMERA_TOPIC, timeout=5.0):
    """
    Grab one frame from camera_topic and return as base64-encoded JPEG string.
    Returns None on failure.
    """
    if not CV_AVAILABLE:
        rospy.logerr('[VLMState] cv_bridge not available')
        return None
    try:
        msg = rospy.wait_for_message(camera_topic, Image, timeout=timeout)
        cv_img = BRIDGE.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        _, buf = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, 85])
        return base64.b64encode(buf.tobytes()).decode('utf-8')
    except Exception as e:
        rospy.logerr('[VLMState] Image grab failed: %s', e)
        return None


def _call_vlm(query_type, image_b64, context=''):
    """Call /vlm/query service. Returns (answer, success)."""
    if not VLM_SRV_AVAILABLE:
        rospy.logerr('[VLMState] hsr_task_sm VLMQuery srv not available')
        return None, False
    try:
        rospy.wait_for_service(VLM_SERVICE, timeout=SERVICE_TIMEOUT)
        proxy = rospy.ServiceProxy(VLM_SERVICE, VLMQuery)
        resp = proxy(query_type=query_type, image_base64=image_b64, context=context)
        return resp.answer, resp.success
    except rospy.ROSException as e:
        rospy.logerr('[VLMState] Service not available: %s', e)
        return None, False
    except Exception as e:
        rospy.logerr('[VLMState] Service call failed: %s', e)
        return None, False


# =============================================================================
# CheckSeatEmpty
# =============================================================================

class CheckSeatEmpty(smach.State):
    """
    Looks at the sofa/seating area and determines which seat(s) are empty.

    Params:
        camera_topic (str): camera topic to grab image from
        seat_key (str): userdata key to store result in (default: 'empty_seat')

    Outcomes:
        left    - only left seat is empty
        right   - only right seat is empty
        both    - both seats empty
        none    - no empty seats
        failed  - could not determine
    """

    def __init__(self, camera_topic=DEFAULT_CAMERA_TOPIC, seat_key='empty_seat'):
        smach.State.__init__(
            self,
            outcomes=['left', 'right', 'both', 'none', 'failed'],
            output_keys=[seat_key],
        )
        self.camera_topic = camera_topic
        self.seat_key = seat_key

    def execute(self, userdata):
        rospy.loginfo('[CheckSeatEmpty] Grabbing image...')
        img = _grab_image_base64(self.camera_topic)
        if img is None:
            return 'failed'

        answer, success = _call_vlm('empty_seat', img)
        if not success or answer not in ('left', 'right', 'both', 'none'):
            rospy.logwarn('[CheckSeatEmpty] Invalid answer: %s', answer)
            return 'failed'

        setattr(userdata, self.seat_key, answer)
        rospy.loginfo('[CheckSeatEmpty] Empty seat: %s', answer)
        return answer


# =============================================================================
# CheckDoorState
# =============================================================================

class CheckDoorState(smach.State):
    """
    Looks at a door (washing machine, dishwasher, room door) and returns open/closed.

    Params:
        camera_topic (str): camera topic
        door_state_key (str): userdata key to store result (default: 'door_state')

    Outcomes:
        open   - door is open
        closed - door is closed
        failed - could not determine
    """

    def __init__(self, camera_topic=DEFAULT_CAMERA_TOPIC, door_state_key='door_state'):
        smach.State.__init__(
            self,
            outcomes=['open', 'closed', 'failed'],
            output_keys=[door_state_key],
        )
        self.camera_topic = camera_topic
        self.door_state_key = door_state_key

    def execute(self, userdata):
        rospy.loginfo('[CheckDoorState] Grabbing image...')
        img = _grab_image_base64(self.camera_topic)
        if img is None:
            return 'failed'

        answer, success = _call_vlm('door_state', img)
        if not success or answer not in ('open', 'closed'):
            rospy.logwarn('[CheckDoorState] Invalid answer: %s', answer)
            return 'failed'

        setattr(userdata, self.door_state_key, answer)
        rospy.loginfo('[CheckDoorState] Door is: %s', answer)
        return answer


# =============================================================================
# GetShelfPlacement
# =============================================================================

class GetShelfPlacement(smach.State):
    """
    Looks at a shelf/cabinet and recommends which shelf to place an object on.

    Params:
        camera_topic (str): camera topic
        object_key (str): userdata key holding the object name (used as context)
        shelf_key (str): userdata key to store result (default: 'shelf_placement')

    Outcomes:
        top    - place on top shelf
        middle - place on middle shelf
        bottom - place on bottom shelf
        failed - could not determine
    """

    def __init__(self, camera_topic=DEFAULT_CAMERA_TOPIC,
                 object_key='grasped_object', shelf_key='shelf_placement'):
        smach.State.__init__(
            self,
            outcomes=['top', 'middle', 'bottom', 'failed'],
            input_keys=[object_key],
            output_keys=[shelf_key],
        )
        self.camera_topic = camera_topic
        self.object_key = object_key
        self.shelf_key = shelf_key

    def execute(self, userdata):
        context = getattr(userdata, self.object_key, 'object')
        rospy.loginfo('[GetShelfPlacement] Object: %s — grabbing image...', context)

        img = _grab_image_base64(self.camera_topic)
        if img is None:
            return 'failed'

        answer, success = _call_vlm('shelf_placement', img, context=str(context))
        if not success or answer not in ('top', 'middle', 'bottom'):
            rospy.logwarn('[GetShelfPlacement] Invalid answer: %s', answer)
            return 'failed'

        setattr(userdata, self.shelf_key, answer)
        rospy.loginfo('[GetShelfPlacement] Shelf: %s', answer)
        return answer
