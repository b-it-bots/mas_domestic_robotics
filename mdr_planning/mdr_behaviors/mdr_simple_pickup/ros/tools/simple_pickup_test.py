#!/usr/bin/env python

import rospy
import mdr_behavior_msgs.srv

if __name__ == '__main__':
    rospy.init_node('pickup_test')

    rospy.loginfo('Waiting for "pickup" service')
    rospy.wait_for_service('/pickup')
    pickup = rospy.ServiceProxy('/pickup', mdr_behavior_msgs.srv.Pickup)
    rospy.loginfo('Found service "pickup"')

    req = mdr_behavior_msgs.srv.PickupRequest()
    req.position.header.frame_id = 'base_link'
    req.position.header.stamp = rospy.Time.now()
    req.position.point.x = -0.7
    req.position.point.y =  0.0
    req.position.point.z =  0.9

    resp = pickup(req)

    if (resp.success == True):
        rospy.loginfo('Pickup succeeded')
    else:
        rospy.loginfo('Pickup failed')
