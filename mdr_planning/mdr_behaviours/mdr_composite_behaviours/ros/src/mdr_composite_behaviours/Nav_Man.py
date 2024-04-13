import cv2
import sys
import time
import math
import tf2_ros
import tf2_geometry_msgs
import rospy

#from optimove_server3 import ArmBase
import roslib
#roslib.load_manifest('my_packge') #???
import actionlib
from visualization_msgs.msg import Marker
import geometry_msgs.msg
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseArray
from move_base_msgs.msg import MoveBaseActionFeedback, MoveBaseActionResult
from moveit_msgs.msg import PickupActionFeedback, PickupActionResult
# from mdr_pickup_action.msg import PickupAction, PickupGoal
#from mas_hsr_gripper_controller.gripper_controller import GripperController
from moveit_commander import MoveGroupCommander 
from actionlib_msgs.msg import GoalID

class Starter:
    def __init__(self):
        self.pipeline_sub = rospy.Subscriber("image_converter/exec_gesture_pipe", String, self.pipeline_callback)
        self.execute_pipeline = -1
    
    def pipeline_callback(self, data):
        if (data.data == "e-start"):
            self.execute_pipeline = 1
        elif (data.data == "e-stop"):
            self.execute_pipeline = 2
        
class Mover:
    def __init__(self):
        self.tfbuffer_ = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tfbuffer_)
        self.pipeline_pub = rospy.Publisher('image_converter/base_poses', PoseArray, queue_size=1)
        self.pipeline_pub_obj = rospy.Publisher('image_converter/object_pose', PoseStamped, queue_size=1)
        self.cancel_goals_pub = rospy.Publisher("/move_base/move/cancel", GoalID, queue_size=10)

        self.pub = rospy.Publisher('object_pose', PoseStamped, queue_size=1)
        self.pose_pub = rospy.Publisher('goal', PoseStamped, queue_size=1)
        for i in range(13):
            globals()["marker_pub_"+str(i)] = rospy.Publisher("/visualization_marker_"+str(i), Marker, queue_size = 10)
        self.goal_achieved = -1
        self.robot_state = -1
        self.rob_cur_cor = PoseStamped()
        # self.pc = rospy.Publisher("cropped_pc", PointCloud2, queue_size=1)
        rospy.Subscriber("/move_base/move/feedback", MoveBaseActionFeedback, self.update_robot_state)
        rospy.Subscriber("/move_base/move/result", MoveBaseActionResult, self.update_goal_achieved)
        rospy.Subscriber("/pickup/feedback", PickupActionFeedback, self.manipulation_state)
        rospy.Subscriber("/pickup/result", PickupActionResult, self.manipulation_complete)
        rospy.Subscriber("/global_pose", PoseStamped, self.subs_base_pose_fn)
        
        # self.arm = MoveGroupCommander("arm", wait_for_servers=0.0)
        # self.arm.set_max_acceleration_scaling_factor(0.4)
        # self.gripper_controller = GripperController()
        # self.head = MoveGroupCommander("head",  wait_for_servers=0.0)
        # self.go_to_home()

    #open3d frame: X=forward-backward, Y=right-left, Z= up-down
    #head_rgbd = X=right-left, Y=down-Up, Z= forward-backward
    #map = X=forward-backward, Y=left-right, Z= up-down

    def transform_head2map(self,obj_poses):
        transformed = []
        for obj_pose in obj_poses:
            transformation = self.tfbuffer_.lookup_transform('map', "head_rgbd_sensor_rgb_frame", rospy.Time())
            transformed_pose = tf2_geometry_msgs.do_transform_pose(obj_pose, transformation)
            transformed.append(transformed_pose)
        return transformed

    def transform_head2base(self,obj_poses):
        transformed = []
        for obj_pose in obj_poses:
            transformation = self.tfbuffer_.lookup_transform('base_link', "head_rgbd_sensor_rgb_frame", rospy.Time())
            transformed_pose = tf2_geometry_msgs.do_transform_pose(obj_pose, transformation)
            #transformed_pose.pose.position.z = 0.92
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
        
    def go_to_home(self):
        arm_target = {'arm_lift_joint': 0.0,
                      'arm_flex_joint': 0.0,
                      'arm_roll_joint': 0.0,
                      'wrist_flex_joint': -math.pi/2.,
                      'wrist_roll_joint': 0.,
                      'wrist_ft_sensor_frame_joint': 0.0}
        self.arm.set_joint_value_target(arm_target)
        self.arm.go()
        '''print("initial arm pose:")
        arm_pos = self.get_arm_cur()
        print(arm_pos.transform.translation.x)
        print(arm_pos.transform.translation.y)
        print(arm_pos.transform.translation.z)'''  
        head_target = {'head_pan_joint': 0.0,
                       'head_tilt_joint': 0.0}
        self.head.set_joint_value_target(head_target)
        self.head.go()
        self.loop_rate.sleep()

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
        start = time.time()
        time_out = 20 #seconds
        #print(start+time_out)
        self.rviz_marker(current,2,mtype=0)
        for i, mark in enumerate(markers):
            self.rviz_marker(mark,i+3+1)
        self.rviz_marker(obj_mark,0)
        self.rviz_marker(rob_mark,1, mtype=0)
        '''while time.time() < start+time_out:
            #print(time.time())
            self.rviz_marker(current,2,mtype=0)
            for i, mark in enumerate(markers):
                self.rviz_marker(mark,i+3+1)
            self.rviz_marker(obj_mark,0)
            self.rviz_marker(rob_mark,1, mtype=0)'''
            #self.rviz_marker(new_rob_mark,2, mtype=0)

    def subs_base_pose_fn(self, data):
        self.rob_cur_cor = data
        #print(self.rob_cur_cor)

    def grab(self,obj_pose):
        #manipulation
        client = actionlib.SimpleActionClient('pickup_server', PickupAction)
        client.wait_for_server()

        goal = PickupGoal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = rospy.Time.now()

        goal.pose.pose.position.x = obj_pose.pose.position.x
        goal.pose.pose.position.y = obj_pose.pose.position.y
        goal.pose.pose.position.z = obj_pose.pose.position.z

        goal.pose.pose.orientation.x = obj_pose.pose.orientation.x
        goal.pose.pose.orientation.y = obj_pose.pose.orientation.y
        goal.pose.pose.orientation.z = obj_pose.pose.orientation.z
        goal.pose.pose.orientation.w = obj_pose.pose.orientation.w

        print(obj_pose)
        print("grabbing......................")
        client.send_goal(goal)
        client.wait_for_result()

    def manipulation_complete(self, msg):
        self.manipulation_completion_status = msg.status.status
    
    def manipulation_state(self, msg):
        self.manipulation_state = msg.status.status

    def update_goal_achieved(self,msg):
        self.goal_achieved = msg.status.status

    def update_robot_state(self, msg):
        self.robot_state = msg.status.status

    def check_goal_reached(self):
        count = 0
        while (True):
            count += 1
            time.sleep(1)
            if (self.robot_state == 1):
                print("Move Base Feedback: Active state")
                break
            elif count == 10:
                print("No Active state for Move Base feedback found in 10 seconds")
                return 2
                
        count = 0
        Wait_time = 40
        while (True):
            count += 1
            time.sleep(1)
            print("Waiting for navigation to complete in "+str(Wait_time)+" seconds. Elapsed sec:"+str(count)+" ")
            if self.goal_achieved==3:
                print("Move Base Feedback: Succeeded goal reached state")
                return 1
            elif count == Wait_time:
                print("Goal reach not possible in "+str(Wait_time)+" seconds after planning")
                return 0

    def check_manipulation_completed(self):
        count = 0
        while (True):
            count += 1
            time.sleep(1)
            if (self.manipulation_state == 1):
                print("Manipulation Move It Feedback: Active state")
                break
            elif count == 10:
                print("Manipulation: No Active state for Move It feedback found in 10 seconds")
                return False
                
        count = 0
        Wait_time = 50
        while (True):
            count += 1
            time.sleep(1)
            print("Waiting for manipulation to complete in "+str(Wait_time)+" seconds. Elapsed sec:"+str(count)+" ")
            if self.manipulation_completion_status==3:
                print("Move Base Feedback: Succeeded goal reached state")
                return True
            elif count == Wait_time:
                print("Picking not completed in "+str(Wait_time)+" seconds after starting")
                return False


    def publisher(self, points):
        self.pub.publish(points)

    def go_to_loc(self,ppose):
        self.pose_pub.publish(ppose)

    def initial_pose_publish(self):
        print('Publishing the initial pose')
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.pose.position.x = 0.5
        initial_pose.pose.position.y = 0.9
        initial_pose.pose.position.z = 0.0
        initial_pose.pose.orientation.x = 0.0
        initial_pose.pose.orientation.y = 0.0
        initial_pose.pose.orientation.z = 0.679
        initial_pose.pose.orientation.w = 0.733
        self.pose_pub.publish(initial_pose)

    def second_pose_publish(self):
        print('Publishing the second pose')
        second_pose = PoseStamped()
        second_pose.header.frame_id = 'map'
        second_pose.pose.position.x = 1.0
        second_pose.pose.position.y = 2.0
        second_pose.pose.position.z = 0.0
        second_pose.pose.orientation.x = 0.0
        second_pose.pose.orientation.y = 0.0
        second_pose.pose.orientation.z = 0.679
        second_pose.pose.orientation.w = 0.733
        self.pose_pub.publish(second_pose)
