#!/usr/bin/env python
import rospy
import tf

class PoseSaver():
    def __init__(self, file_name, map_frame = 'map', robot_frame='base_link'):
        self.file_name = file_name
        self.map_frame = map_frame
        self.robot_frame = robot_frame
        tf_received = False
        self.pose_name = "Default"

    def set_pose_name(self, pose_name):
        self.pose_name = pose_name

    def get_pose_name(self):
        return self.pose_name

    def is_exit_requested(self):
        pass

    def save_pose(self):
        tf_received = False
        rospy.loginfo("Saving Pose %s ", self.get_pose_name())
        # get transformation between map and base_link
        while(not tf_received):
            tf_listener = tf.TransformListener()

            try:
                tf_listener.waitForTransform(self.map_frame, self.robot_frame, rospy.Time.now(), rospy.Duration(1))
                (trans, rot) = tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0));

                (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(rot)

                pose_description = "%s: [%lf, %lf, %lf]\n" % (self.pose_name, trans[0], trans[1], yaw)
                rospy.loginfo("Pose Description %s", pose_description)
                tf_received = True
            except Exception, e:
                rospy.sleep(1)
                tf_received = False

        #write position into a file
        if tf_received:
            pose_file = open(self.file_name, 'a')
            pose_file.write(pose_description)
            pose_file.close()

        tf_received = False
