#!/usr/bin/env python

import rospy
import mdr_behavior_msgs.srv

if __name__ == '__main__':
    rospy.init_node('place_test')

    rospy.loginfo('Waiting for "place" service')
    rospy.wait_for_service('/place')
    place = rospy.ServiceProxy('/place', mdr_behavior_msgs.srv.Place)
    rospy.loginfo('Found service "place"')

    req = mdr_behavior_msgs.srv.PlaceRequest()
    req.position.header.frame_id = 'base_link'
    req.position.header.stamp = rospy.Time.now()
    req.position.point.x = -0.7
    req.position.point.y =  0.0
    req.position.point.z =  0.9

    resp = place(req)

    if (resp.success == True):
        rospy.loginfo('Place succeeded')
    else:
        rospy.loginfo('Place failed')
