#!/usr/bin/env python
"""
This module contains components that evaluate a grasp using
a Schunk Dexterous Hand (SDH-2).

"""
#-*- encoding: utf-8 -*-
__author__ = 'jsanch'

import rospy
import std_msgs.msg
import schunk_sdh.msg
import mdr_tactile_tools_common.tactile_filters as tactile_filters


class TactileGraspEvaluator(object):
    """
    Components that evaluate a grasp using the tactile sensors of an SDH-2.

    """
    def __init__(self):
        # params
        self.grasp_evaluator_event = None
        self.grasp_type = None
        self.tactile_data = None

        # Time interval to acquire data.
        self.wait_for_data = rospy.get_param('~wait_for_data')
        # Time interval to publish data.
        self.publish_rate = rospy.get_param('~publish_rate')
        # Mean value to detect a contact in a tactile sensor
        # (in pressure units: Weiss scale).
        self.contact_threshold = rospy.get_param('~contact_threshold')

        # publishers
        self.pub_grasp_status = rospy.Publisher(
            "~event_out", std_msgs.msg.String)

        # subscribers
        rospy.Subscriber("~event_in",
                         std_msgs.msg.String, self.start_grasp_evaluator_cb)
        rospy.Subscriber("~configuration",
                         std_msgs.msg.String, self.configure_grasp_evaluator_cb)
        rospy.Subscriber("~tactile_data", schunk_sdh.msg.TactileSensor,
                         self.read_tactile_data_cb)

    def evaluate_grasp(self):
        """
        Evaluates if an object is being grasped based on the
        specified grasp type.

        """
        rospy.loginfo("Grasp evaluator ready to start...")
        state = 'INIT'

        while not rospy.is_shutdown():

            if state == 'INIT':
                if self.tactile_data:
                    state = 'IDLE'
            elif state == 'IDLE':
                if self.grasp_evaluator_event == 'e_start':
                    state = 'RUNNING'
            elif state == 'RUNNING':
                number_of_sensors = len(self.tactile_data.tactile_matrix)
                tactile_info = [[] for _ in range(number_of_sensors)]

                # convert tactile data into arrays
                for i in range(number_of_sensors):
                    for j in range(len(
                            self.tactile_data.tactile_matrix[i].tactile_array)):
                        tactile_info[i].append(
                            self.tactile_data.tactile_matrix[i].
                            tactile_array[j])

                # get contacts on each tactile sensor
                contact_thresholds = \
                    [self.contact_threshold] * number_of_sensors
                contact_data = tactile_filters.detect_contact(
                    tactile_info, contact_thresholds)

                grasp_status = calculate_grasp_status(self.grasp_type,
                                                      contact_data)

                self.pub_grasp_status.publish(grasp_status)

                if self.grasp_evaluator_event == 'e_stop':
                    state = 'IDLE'

            rospy.sleep(self.publish_rate)

    def configure_grasp_evaluator_cb(self, msg):
        """
        Obtains the specified grasp configuration.

        """
        self.grasp_type = msg.data

    def start_grasp_evaluator_cb(self, msg):
        """
        Starts a grasp evaluator based on the specified type of grasp.

        """
        self.grasp_evaluator_event = msg.data

    def read_tactile_data_cb(self, msg):
        """
        Obtains the tactile information for each phalanx.

        """
        self.tactile_data = msg


def calculate_grasp_status(grasp, contacts):
    """
    Calculates the status of a specified grasp based on a set of contacts.

    Args:
        grasp: String
            The grasp configuration.
        contacts: Int[]
            The tactile sensors that have a contact.
    Returns:
        grasp_status: String
            The status of a specific grasp:
                - 'e_grasped': The object is being grasped.
                - 'e_not_grasped': The object is not being grasped.
                - 'e_failed': Default value when the status of the
                              grasp cannot be determined.

    """
    grasp_status = 'e_failed'

    if grasp == 'cylindrical':
        rospy.logdebug('[%s] grasp evaluator started.' % grasp)
        if evaluate_cylindrical_grasp(contacts):
            grasp_status = 'e_grasped'
        else:
            grasp_status = 'e_not_grasped'

    elif grasp == 'spherical':
        rospy.logdebug('[%s] grasp evaluator started.' % grasp)
        if evaluate_spherical_grasp(contacts):
            grasp_status = 'e_grasped'
        else:
            grasp_status = 'e_not_grasped'

    elif grasp == 'precision':
        rospy.logdebug('[%s] grasp evaluator started.' % grasp)
        if evaluate_precision_grasp(contacts):
            grasp_status = 'e_grasped'
        else:
            grasp_status = 'e_not_grasped'

    elif grasp:
        rospy.logwarn("'%s' is not a supported grasp. Supported grasps "
                      "are: 'cylindrical', 'spherical' and 'precision'."
                      % grasp)

    return grasp_status


def evaluate_cylindrical_grasp(contacts):
    """
    Verifies if an object has been grasped assuming a cylindrical grasp.
    A grasp is assumed if at least one tactile sensor in each side has a
    contact.

    Args:
        contacts: Int[]
            The tactile sensors that have a contact.
    Returns:
        grasp_status: Bool
            The status of the grasp. A 'True' value is returned when the
            object is being grasped.

    """
    if (contacts[0] or contacts[1]) and\
       (contacts[2] or contacts[3] or contacts[4] or contacts[5]):
        return True
    else:
        return False


def evaluate_spherical_grasp(contacts):
    """
    Verifies if an object has been grasped assuming a spherical grasp.
    A grasp is assumed if at least one tactile sensor in each finger has a
    contact.

    Args:
        contacts: Int[]
            The tactile sensors that have a contact.
    Returns:
        grasp_status: Bool
            The status of the grasp. A 'True' value is returned when the
            object is being grasped.

    """
    if (contacts[0] or contacts[1]) and (contacts[2] or contacts[3])\
       and (contacts[4] or contacts[5]):
        return True
    else:
        return False


def evaluate_precision_grasp(contacts):
    """
    Verifies if an object has been grasped assuming a precision grasp.
    A grasp is assumed if both fingertips have a contact.

    Args:
        contacts: Int[]
            The tactile sensors that have a contact.
    Returns:
        grasp_status: Bool
            The status of the grasp. A 'True' value is returned when the
            object is being grasped.

    """
    if contacts[3] and contacts[5]:
        return True
    else:
        return False


def main():
    rospy.init_node("grasp_evaluators", anonymous=True)
    tactile_grasp_evaluator = TactileGraspEvaluator()
    tactile_grasp_evaluator.evaluate_grasp()
    rospy.spin()
