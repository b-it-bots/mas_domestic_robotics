from importlib import import_module
from PIL import Image as PILImage
import yaml

import rospy
import smach
import torch

import tf
from sensor_msgs.msg import PointCloud2, Image
from mdr_find_people.msg import FindPeopleResult
from mas_perception_msgs.msg import Person, PersonList, ObjectView
from mas_perception_libs import ImageDetectionKey
from mas_perception_libs.visualization import crop_image
from mas_perception_libs.utils import cloud_msg_to_cv_image
from cv_bridge import CvBridge
from mdr_find_people.find_people import FindPeople

from dataset_interface.siamese_net.model import SiameseNetwork
from dataset_interface.siamese_net.utils import get_transforms

class FindPeopleState(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed'],
                             input_keys=['find_people_goal'],
                             output_keys=['find_people_result', 'error_message'])

        self._listener = tf.TransformListener()
        self.pointcloud_topic = rospy.get_param("~pointcloud_topic", '/rectified_points')
        self.class_annotation_file = rospy.get_param('class_annotation_file', '')
        self.detector_module = rospy.get_param('detector_module', 'torchvision.models.detection')
        self.detector_name = rospy.get_param('detector_name', 'fasterrcnn_resnet50_fpn')
        self.detection_threshold = float(rospy.get_param('detection_threshold', 0.))
        self.face_embedding_model_path = rospy.get_param("~face_embedding_model_path", '')

        if not self.class_annotation_file:
            raise ValueError('Parameter "class_annotation_file" cannot be empty!')

        self.class_annotations = None
        with open(self.class_annotation_file, 'r') as annotations_file:
            self.class_annotations = yaml.safe_load(annotations_file)
        self.class_annotations[0] = 'background'

        if not self.detector_module:
            rospy.logwarn('[find_people] Found empty "detector_module"; using default torchvision.models.detection')
            self.detector_module = 'torchvision.models.detection'

        if not self.detector_name:
            rospy.logwarn('[find_people] Found empty "detector_name"; using default fasterrcnn_resnet50_fpn')
            self.detector_module = 'fasterrcnn_resnet50_fpn'

        self.model_device = torch.device('cuda') if torch.cuda.is_available() \
                                                 else torch.device('cpu')
        detector_class = getattr(import_module(self.detector_module),
                                 self.detector_name)
        self.detector = detector_class(pretrained=True)
        self.detector.eval()
        self.detector.to(self.model_device)

        self.face_embedding_model = None
        if self.face_embedding_model_path:
            self.face_embedding_model = SiameseNetwork()
            self.face_embedding_model.load_state_dict(torch.load(self.face_embedding_model_path))
            self.face_embedding_model.eval()
            self.face_embedding_model.to(self.model_device)

        self.cv_bridge = CvBridge()

    def execute(self, userdata):
        rospy.loginfo('Executing state FIND_PEOPLE')

        # Get the pointcloud
        cloud_msg = rospy.wait_for_message(self.pointcloud_topic, PointCloud2)

        # Get positions of people
        predictions, bb2ds, poses = FindPeople.detect(cloud_msg,
                                                      self.detector,
                                                      self.model_device,
                                                      self.class_annotations,
                                                      self.detection_threshold)

        # Get people images
        cv_image = cloud_msg_to_cv_image(cloud_msg)
        images = []
        face_images = []
        for i, bb2d in enumerate(bb2ds):
            cropped_cv = crop_image(cv_image, bb2d)
            cropped_img_msg = self.cv_bridge.cv2_to_imgmsg(cropped_cv, encoding="passthrough")

            rospy.loginfo('[find_people] Attempting to extract face of person {0}'.format(i+1))
            cropped_face_img = FindPeople.extract_face_image(cropped_cv)
            if cropped_face_img is not None:
                cropped_face_img_msg = self.cv_bridge.cv2_to_imgmsg(cropped_face_img,
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

            person_view = ObjectView()
            person_view.image = images[i]
            p.views.append(person_view)

            face_view = ObjectView()
            face_view.image = face_images[i]

            if self.face_embedding_model is not None and face_view.image.data:
                face_cv2 = self.cv_bridge.imgmsg_to_cv2(face_view.image)
                img_tensor = get_transforms()(PILImage.fromarray(face_cv2))
                img_tensor.unsqueeze_(0)

                embedding = self.face_embedding_model.forward_once(img_tensor)
                face_view.embedding.embedding = embedding.detach().numpy().squeeze().tolist()

            p.face.views.append(face_view)
            pl.append(p)

        # Package that actual PersonList message
        person_list = PersonList()
        person_list.persons = pl
        result = FindPeopleResult()
        result.person_list = person_list

        userdata['find_people_result'] = result
        return 'succeeded'
