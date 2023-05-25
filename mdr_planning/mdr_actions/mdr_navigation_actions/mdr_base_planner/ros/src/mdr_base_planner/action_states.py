#!/usr/bin/python
import rospy
import cv2
import numpy as np
import ros_numpy
import time
from mdr_base_planner.planner import t2d2t3d
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pyftsm.ftsm import FTSMTransitions
from mas_execution.action_sm_base import ActionSMBase
from mdr_detect_gesture.msg import DetectGestureResult
from mdr_base_planner.msg import BasePlannerResult
from mdr_perception_msgs.msg import BodyBoundingBox
from collections import deque
import tf2_ros
from visualization_msgs.msg import Marker
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2#, Image as ImageMsg
from geometry_msgs.msg import PoseStamped



class BasePlannerSM(ActionSMBase):
    def __init__(self, timeout=120.,
                 planner_topic = 'heartmet/pose_planner',
                 cloud_topic = '/hsrb/head_rgbd_sensor/depth_registered/rectified_points',
                 map_img_path='/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_actions/mdr_navigation_actions/mdr_base_planner/ros/map/map.pgm',   
                 max_recovery_attempts=3):
        super(BasePlannerSM, self).__init__('PosePlanner', [], max_recovery_attempts)
        self.timeout = timeout
        self.bridge = CvBridge()
        self.num_person = 1
        self.image = Image()
        #rospy.init_node('base_planner')
        self.tfbuffer_ = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tfbuffer_)
        for i in range(15):
            globals()["marker_pub_"+str(i)] = rospy.Publisher("/visualization_marker_"+str(i), Marker, queue_size = 10)
        self.map_img = cv2.imread(map_img_path)
        self.map_grey = cv2.cvtColor(self.map_img, cv2.COLOR_BGR2GRAY)
        self.rob_cur_cor = PoseStamped()
        self.bbox = None
        rospy.Subscriber("/global_pose", PoseStamped, self.subs_base_pose_fn)
        rospy.Subscriber("/mas_perception/detection_result", BodyBoundingBox, self.bbox_call_back)
        self.cloud_sub = rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", PointCloud2, self.callback1)


    def init(self):
        # try:
        #     rospy.loginfo('[detect_person] Loading detection model %s', self.detection_model_path)
        #     self.model = MultiPerDetector(self.rp, num_person = self.num_person, model_path='/home/zany/catkin_hsr_ws/src/mas_domestic_robotics/mdr_planning/mdr_actions/mdr_perception_actions/mdr_detect_gesture/model/action10_9_np3.tflite')
        # except Exception as exc:
        #     rospy.logerr('[detect_gesture] Model %s could not be loaded: %s',
        #                  self.detection_model_path, str(exc))
        
        return FTSMTransitions.INITIALISED


    def running(self):

        while self.result == None:
            self.base_plan()
        # gest_sub.unregister()
        
        return FTSMTransitions.DONE
    
    def base_plan(self):
        self.bounding_box = BodyBoundingBox()
        # bbox = self.bbox #type error fix TODO
        bbox = [263, 313, 320, 410]
        td23D = t2d2t3d("/home/zany/catkin_hsr_ws/src/mas_domestic_robotics/mdr_planning/mdr_actions/mdr_navigation_actions/mdr_base_planner/ros/map/map.pgm")
        res =  td23D.plan_grab_poses(self.cloud_data, self.transform_3D2head, self.transform_head2map, self.rob_cur_cor, bbox, viz=False)
        if res:
            object_pose, coordinates_array = res[0], res[1]
            base_pose_msg = BasePlannerResult()
            base_pose_msg.pose = object_pose
            base_pose_msg.posearray = coordinates_array
            self.result = base_pose_msg

            if coordinates_array:
                self.visualize_rviz(coordinates_array,object_pose,self.rob_cur_cor)

    
    def bbox_call_back(self,data):
        self.bbox = data

    def subs_base_pose_fn(self, data):
        self.rob_cur_cor = data

    def callback1(self,data):
        #print('callback')
        try:
            self.cloud_data = data
            #print(self.cv_image)
        except ValueError as e:
            print(e)

    def create_marker(self,cor_pose,r,g,b):
        marker = Marker()
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.pose.position.x = cor_pose.pose.position.x
        marker.pose.position.y = cor_pose.pose.position.y
        marker.pose.position.z = cor_pose.pose.position.z
        marker.pose.orientation.x = cor_pose.pose.orientation.x
        marker.pose.orientation.y = cor_pose.pose.orientation.y
        marker.pose.orientation.z = cor_pose.pose.orientation.z
        marker.pose.orientation.w = cor_pose.pose.orientation.w
        return marker

    def rviz_marker(self, mkr, idx, mtype=2):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = mtype
        marker.id = 0
        # Set the scale of the marker
        marker.scale.x = 0.1
        if mtype==0:
            marker.scale.x = 0.3
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        # Set the color
        marker.color.r = mkr.color.r
        marker.color.g = mkr.color.g
        marker.color.b = mkr.color.b
        marker.color.a = 1.0
        # Set the pose of the marker
        marker.pose.position.x = mkr.pose.position.x
        marker.pose.position.y = mkr.pose.position.y
        marker.pose.position.z = mkr.pose.position.z
        marker.pose.orientation.x = mkr.pose.orientation.x
        marker.pose.orientation.y = mkr.pose.orientation.y
        marker.pose.orientation.z = mkr.pose.orientation.z
        marker.pose.orientation.w = mkr.pose.orientation.w
        #print("marker before publish")
        globals()["marker_pub_"+str(idx)].publish(marker)
        #print("marker after publish")
        #rospy.rostime.wallsleep(1.0)

    def valid_create_marker(self,cor_pose,r,g,b):
        marker = Marker()
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.pose.position.x = cor_pose.position.x
        marker.pose.position.y = cor_pose.position.y
        marker.pose.position.z = cor_pose.position.z
        marker.pose.orientation.x = cor_pose.orientation.x
        marker.pose.orientation.y = cor_pose.orientation.y
        marker.pose.orientation.z = cor_pose.orientation.z
        marker.pose.orientation.w = cor_pose.orientation.w
        return marker

    def visualize_rviz(self, valid_points, real_obj_pose, base_pose_robot):#, new_loc):
        markers=[]
        current = self.valid_create_marker(valid_points.poses[0],0.0,1.0,0.0)
        for q in range(len(valid_points.poses)-1):
            mark = self.valid_create_marker(valid_points.poses[q+1],1.0,0.0,0.0)
            markers.append(mark)
        obj_mark = self.create_marker(real_obj_pose,0.0,0.0,1.0)
        rob_mark = self.create_marker(base_pose_robot,0.0,1.0,1.0)
        #new_rob_mark = self.create_marker(new_loc,0.0,1.0,0.0)
        self.rviz_marker(current,2,mtype=0)
        for i, mark in enumerate(markers):
            self.rviz_marker(mark,i+3+1)
        self.rviz_marker(obj_mark,0)
        self.rviz_marker(rob_mark,1, mtype=0)
        #self.rviz_marker(new_rob_mark,2, mtype=0)
    
    def transform_head2map(self,obj_poses):
        transformed = []
        for obj_pose in obj_poses:
            transformation = self.tfbuffer_.lookup_transform('map', "head_rgbd_sensor_rgb_frame", rospy.Time())
            transformed_pose = tf2_geometry_msgs.do_transform_pose(obj_pose, transformation)
            transformed.append(transformed_pose)
        return transformed

    def transform_3D2head(self,obj_poses):
        transformed = []
        for obj_pose in obj_poses:
            x,y,z = obj_pose.pose.position.x, obj_pose.pose.position.y, obj_pose.pose.position.z
            obj_pose.header.frame_id = "head_rgbd_sensor_rgb_frame"
            obj_pose.pose.position.x = y #headx=3dy
            obj_pose.pose.position.y = z #heady=-3dz
            obj_pose.pose.position.z = x #headz=3dx
            transformed.append(obj_pose)
        return transformed
