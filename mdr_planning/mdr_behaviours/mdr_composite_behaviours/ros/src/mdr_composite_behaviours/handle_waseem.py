import rospy
import tf2_ros
import tf
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
rospy.init_node('coordinate_transform_node')
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
#tf_listener = tf.TransformListener()
# Create a PointStamped message for the object's position in the camera frame
object_camera_point = PoseStamped()
object_camera_point.header.frame_id = "head_rgbd_sensor_rgb_frame"  # Replace with your camera frame ID
object_camera_point.header.stamp = rospy.Time.now()  # Use the latest available transform
# Set the object's coordinates in the camera frame
object_camera_point.pose.position.x = 0.126
object_camera_point.pose.position.y = 0.027
object_camera_point.pose.position.z = 0.686

# x=0.126, y=0.027, z=0.686
rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    try:
        # Transform the object's position to the base_link frame
        object_base_link_point = tf_buffer.lookup_transform("head_rgbd_sensor_rgb_frame", "map", rospy.Time())
        transform_pose = tf2_geometry_msgs.do_transform_pose(object_camera_point, object_base_link_point)
        #object_base_link_point = tf_listener.transformPose("base_link", object_camera_point)
        # Extract the transformed coordinates
        #base_link_x = object_base_link_point.point.x
        #base_link_y = object_base_link_point.point.y
        #base_link_z = object_base_link_point.point.z
        print(transform_pose)
        # Now, base_link_x, base_link_y, and base_link_z contain the object's coordinates in the base_link frame.
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn(f"Transform failed: {e}")
        rate.sleep()
        continue
    rate.sleep()