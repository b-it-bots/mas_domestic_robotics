import numpy as np
from sensor_msgs.msg import PointCloud2, Image
from sklearn.impute import SimpleImputer
import torch
import ros_numpy
import cv2
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from mas_perception_libs.image_detector import ImageDetectionKey, ImageDetectorBase, TorchImageDetector
from mas_perception_libs.utils import cloud_msg_to_cv_image, crop_cloud_to_xyz, crop_organized_cloud_msg
from mas_perception_libs.visualization import draw_labeled_boxes
from cv_bridge import CvBridge, CvBridgeError
import dataset_interface.object_detection.transforms as T
import pdb


class MediapipeFaceDetector(object):
    @staticmethod
    def face_detector(cloud_msg, faceDetection):
        rospy.loginfo('Running detector...')
        cv_image = cloud_msg_to_cv_image(cloud_msg).astype('uint8')
        results = faceDetection.process(cv_image)
        predictions = []
        img = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        if results.detections:
            img = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            for id, detection in enumerate(results.detections):
                bboxC = detection.location_data.relative_bounding_box
                ih, iw, ic = img.shape
                bbox = (int(bboxC.xmin * iw), int(bboxC.ymin * ih),
                        int(bboxC.width * iw), int(bboxC.height * ih))
                cv2.rectangle(img, bbox, (255, 0, 255), 2)
                score_text = str(int(detection.score[0] * 100)) + '%'
                cv2.putText(img, score_text,
                            (bbox[0], bbox[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 255), 2)
                x_min = bboxC.xmin * iw
                y_min = bboxC.ymin * ih
                fw = bboxC.width * iw
                fh = bboxC.height * ih
                x_max = x_min + fw
                y_max = y_min + fh
                predictions.append({
                    ImageDetectionKey.CLASS: 'person',
                    ImageDetectionKey.CONF: float(detection.score[0]),
                    ImageDetectionKey.X_MIN: float(x_min),
                    ImageDetectionKey.Y_MIN: float(y_min),
                    ImageDetectionKey.X_MAX: float(x_max),
                    ImageDetectionKey.Y_MAX: float(y_max),
                })
        return predictions, img

    @staticmethod
    def detect(cloud_msg, faceDetection):
        rospy.loginfo('Processing detections...')
        predictions, detect_image = MediapipeFaceDetector.face_detector(cloud_msg, faceDetection)
        rospy.loginfo('Extracting bounding boxes...')
        bb2ds = ImageDetectorBase.prediction_to_bounding_boxes(predictions)[0]
        rospy.loginfo('Extracting people detections...')
        predictions, bb2ds = MediapipeFaceDetector.filter_people(predictions, bb2ds)
        if len(predictions) == 1:
            rospy.loginfo('Found one person')
        else:
            rospy.loginfo('Found {0} people'.format(len(predictions)))
        poses = []
        if predictions:
            rospy.loginfo('Extracting person poses...')
            poses = MediapipeFaceDetector.get_people_poses(cloud_msg, predictions, bb2ds)
        rospy.loginfo('Person detection complete')
        return predictions, bb2ds, poses, detect_image

    @staticmethod
    def filter_people(predictions, bounding_boxes):
        people_preds = []
        people_bbs = []
        for i, _ in enumerate(predictions):
            pred = predictions[i]
            bb2d = bounding_boxes[i]
            if pred[ImageDetectionKey.CLASS] == 'person':
                people_preds.append(pred)
                people_bbs.append(bb2d)
        return people_preds, people_bbs

    @staticmethod
    def get_people_poses(cloud_msg, predictions, bounding_boxes):
        poses = []
        for i, _ in enumerate(predictions):
            bb2d = bounding_boxes[i]
            obj_coords = crop_cloud_to_xyz(cloud_msg, bb2d)
            obj_coords_pc = PointCloud2()
            obj_coords_pc = crop_organized_cloud_msg(cloud_msg, bb2d)
            rospy.loginfo('[find_people] publishing person filtered cloud')
            rospy.loginfo('[find_people] obj_coord data type: {}'.format(type(obj_coords_pc)))
            init_pub = rospy.Publisher('/filtered_people_point_cloud', PointCloud2, queue_size=10)
            rospy.sleep(3)
            init_pub.publish(obj_coords_pc)
            rospy.sleep(1)
            imputer = SimpleImputer(missing_values=np.nan, strategy='most_frequent')
            imputer.fit(obj_coords.reshape(obj_coords.shape[0]*obj_coords.shape[1], obj_coords.shape[2]))
            obj_coords_without_nans = imputer.transform(obj_coords.reshape(obj_coords.shape[0]*obj_coords.shape[1], obj_coords.shape[2]))
            mean_coords = np.nanmean(obj_coords_without_nans.astype(np.float32), axis=0)
            mean_ps = PoseStamped()
            mean_ps.header = Header(frame_id=cloud_msg.header.frame_id)
            mean_ps.pose = Pose(Point(mean_coords[0], mean_coords[1], mean_coords[2]), Quaternion())
            poses.append(mean_ps)
        return poses

    @staticmethod
    def render_image_with_detections(cloud_msg, bounding_boxes):
        cv_image = cloud_msg_to_cv_image(cloud_msg)
        image = draw_labeled_boxes(cv_image, bounding_boxes)
        return image

    @staticmethod
    def extract_face_image(image_array):
        try:
            top, right, bottom, left = face_recognition.face_locations(image_array)[0]
            rospy.loginfo('[find_people] Successfully extracted face from person image.')
            return image_array[top:bottom, left:right]
        except IndexError:
            rospy.logwarn('[find_people] Failed to extract face from person image!')
            return None
