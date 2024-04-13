#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

def callback(data):
    # Find the index of the hand_motor_joint
    try:
        index = data.name.index('hand_motor_joint')
        hand_motor_joint_value = data.position[index]
        print("hand_motor_joint Value: ", hand_motor_joint_value)
    except ValueError:
        pass  # Do nothing if the joint is not found

def listener():
    rospy.init_node('hand_motor_joint_listener', anonymous=True)
    rospy.Subscriber("/hsrb/joint_states", JointState, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
