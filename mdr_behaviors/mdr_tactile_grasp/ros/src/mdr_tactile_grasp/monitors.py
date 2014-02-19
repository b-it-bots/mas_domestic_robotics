#!/usr/bin/env python
"""
This module contains components that monitor the robot's state, e.g.
using tactile information, joints values.

"""
#-*- encoding: utf-8 -*-
__author__ = 'jsanch'

import rospy
import std_msgs.msg
import schunk_sdh.msg
import control_msgs.msg
import mdr_tactile_grasp_common.tactile_data_computation as tactile_computation
import mdr_tactile_grasp_common.joints_computation as joints_computation


class TactileMonitors(object):
    """
    Components that monitor the tactile sensors of an SDH-2.

    """
    def __init__(self):
        # params
        self.tactile_info = None
        self.contact_thresholds = None
        self.matrix_widths = None           # cells_x
        self.matrix_lengths = None          # cells_y

        # ToDo: Make these config params
        self.wait_for_data = 0.2     # seconds
        self.publish_rate = 0.1      # seconds
        self.contact_detection_value = 10   # pressure units (Weiss scale)

        # publishers
        self.pub_threshold_detector = rospy.Publisher(
            "e_threshold_detector_status", std_msgs.msg.String,
            latch=True)
        self.pub_static_contact_detector = rospy.Publisher(
            "e_static_contact_detector_status", std_msgs.msg.String,
            latch=True)
        self.pub_dynamic_contact_detector = rospy.Publisher(
            "e_dynamic_contact_detector_status", std_msgs.msg.String,
            latch=True)
        self.pub_region_extractor = rospy.Publisher(
            "e_region_extractor_status", std_msgs.msg.String,
            latch=True)
        self.pub_static_detected_contacts = rospy.Publisher(
            "mdr_tactile_grasp_static_contacts_detected",
            std_msgs.msg.UInt16MultiArray)
        self.pub_stop_criteria = rospy.Publisher(
            "mdr_tactile_grasp_exceeded_contacts",
            std_msgs.msg.UInt16MultiArray)
        self.pub_contact_regions = rospy.Publisher(
            "mdr_tactile_grasp_contact_regions",
            schunk_sdh.msg.TactileSensor)
        self.pub_contact_forces = rospy.Publisher(
            "mdr_tactile_grasp_contact_forces",
            schunk_sdh.msg.TactileSensor)
        self.pub_contact_locations = rospy.Publisher(
            "mdr_tactile_grasp_contact_locations",
            std_msgs.msg.Int16MultiArray)

        # subscribers
        rospy.Subscriber("e_threshold_detector_start",
                         std_msgs.msg.String, self.start_threshold_detector_cb)
        rospy.Subscriber("e_static_contact_detector_start",
                         std_msgs.msg.String,
                         self.start_static_contact_detector)
        rospy.Subscriber("e_dynamic_contact_detector_start",
                         std_msgs.msg.String,
                         self.start_dynamic_contact_detector)
        rospy.Subscriber("e_contact_region_extractor_start",
                         std_msgs.msg.String,
                         self.start_region_extractor_cb)
        rospy.Subscriber("/dsa_controller/tactile_data",
                         schunk_sdh.msg.TactileSensor,
                         self.raw_tactile_data_cb)
        rospy.Subscriber("mdr_tactile_grasp_contact_thresholds",
                         std_msgs.msg.UInt16MultiArray,
                         self.contact_thresholds_cb)

    def start_threshold_detector_cb(self, msg):
        """
        Starts a 'monitor' component that detects if the contact
        thresholds have been exceeded.
        NB: This is a 'dynamic component', i.e. it keeps executing
        until a stopping criteria is met.

        """
        rospy.loginfo("Received '%s' message. Starting threshold_detector."
                      % msg.data)
        status_check = "e_failed"

        while not (rospy.is_shutdown() or (self.tactile_info
                                           and self.contact_thresholds)):
            rospy.sleep(self.wait_for_data)

        stopping_commands = std_msgs.msg.UInt16MultiArray()

        # keeps updating the stopping_commands (since 'self.tactile_info' is
        # changing continuously) until all joints are supposed to stop.
        while True:
            stopping_commands.data = tactile_computation.detect_threshold(
                self.tactile_info, self.contact_thresholds)
            self.pub_stop_criteria.publish(stopping_commands)
            rospy.sleep(self.publish_rate)

            if sum(stopping_commands.data) == 0:
                status_check = "e_success"
                break

        self.pub_threshold_detector.publish(status_check)

    def start_static_contact_detector(self, msg):
        """
        Starts a 'monitor' component that detects if there is a contact.
        NB: This is a 'static component', i.e. it only executes one time.

        """
        rospy.loginfo("Received '%s' message. Starting contact detector..."
                      % msg.data)

        while not (rospy.is_shutdown() or (self.tactile_info
                                           and self.contact_thresholds)):

            rospy.sleep(self.wait_for_data)

        contact_data = tactile_computation.detect_contact(
            self.tactile_info, self.contact_thresholds)

        contact_info = std_msgs.msg.UInt16MultiArray()
        contact_info.data = contact_data

        if contact_data:
            status_check = "e_success"
            rospy.loginfo("Publishing contact data obtained by the 'contact "
                          "detector'...")
            self.pub_static_detected_contacts.publish(contact_info)
        else:
            status_check = "e_failed"

        self.pub_static_contact_detector.publish(status_check)

    def start_dynamic_contact_detector(self, msg):
        """
        Starts a 'monitor' component that detects if there is a contact.
        NB: This is a 'dynamic component', i.e. it keeps executing
        until a stopping criteria is met.

        """
        rospy.loginfo("Received '%s' message. Starting 'dynamic contact "
                      "detector'..." % msg.data)
        status_check = "e_failed"

        while not (rospy.is_shutdown() or self.tactile_info):
            rospy.sleep(self.wait_for_data)

        stopping_commands = std_msgs.msg.UInt16MultiArray()

        contact_detection_values = \
            [self.contact_detection_value] * len(self.tactile_info)

        # keeps updating the stopping_commands (since 'self.tactile_info' is
        # changing continuously) until all joints are supposed to stop.
        while True:
            stopping_commands.data = tactile_computation.detect_threshold(
                self.tactile_info, contact_detection_values)
            self.pub_stop_criteria.publish(stopping_commands)
            rospy.sleep(self.publish_rate)

            if sum(stopping_commands.data) == 0:
                status_check = "e_success"
                break

        self.pub_dynamic_contact_detector.publish(status_check)

    def start_region_extractor_cb(self, msg):
        """
        Starts a 'monitor' component that extracts the contact regions
        of each phalanx.
        NB: This is a 'static component', i.e. it only executes one time.

        """
        rospy.loginfo("Received '%s' message. Starting contact region "
                      "extractor..." % msg.data)

        while not (rospy.is_shutdown() or (self.tactile_info
                                           and self.contact_thresholds
                                           and self.matrix_widths)):
            rospy.sleep(self.wait_for_data)

        # ToDo: Move this to function(s)
        filtered_tactile_data = tactile_computation.threshold_contact_matrices(
            self.tactile_info, self.contact_thresholds)

        connected_contact_regions = tactile_computation.label_regions(
            filtered_tactile_data, self.matrix_widths)

        contact_regions = tactile_computation.extract_contact_region(
            connected_contact_regions)

        contact_values = tactile_computation.calculate_contact_values(
            self.tactile_info, contact_regions)

        centroids_locations = std_msgs.msg.Int16MultiArray()
        centroids_locations.data = tactile_computation.locate_contact_position(
            contact_values, self.matrix_widths)

        # create empty tactile sensor messages
        largest_contact_regions = schunk_sdh.msg.TactileSensor()
        largest_contact_regions.header.stamp = rospy.Time.now()

        contact_values_matrix = schunk_sdh.msg.TactileSensor()
        contact_values_matrix.header.stamp = rospy.Time.now()

        tactile_matrix = [[] for _, _ in enumerate(contact_regions)]
        for i, _ in enumerate(contact_regions):
            tactile_matrix[i] = \
                schunk_sdh.msg.TactileMatrix()
            largest_contact_regions.tactile_matrix.append(tactile_matrix[i])

        pressure_matrix = [[] for _, _ in enumerate(contact_values)]
        for i, _ in enumerate(contact_values):
            pressure_matrix[i] = \
                schunk_sdh.msg.TactileMatrix()
            contact_values_matrix.tactile_matrix.append(pressure_matrix[i])

        # assign the values to the tactile sensor messages
        for i, _ in enumerate(contact_regions):
            largest_contact_regions.tactile_matrix[i].cells_x = \
                self.matrix_widths[i]
            largest_contact_regions.tactile_matrix[i].cells_y = \
                self.matrix_lengths[i]
            largest_contact_regions.tactile_matrix[i].matrix_id = i
            largest_contact_regions.tactile_matrix[i].tactile_array = \
                contact_regions[i]

        for i, _ in enumerate(contact_values):
            contact_values_matrix.tactile_matrix[i].cells_x = \
                self.matrix_widths[i]
            contact_values_matrix.tactile_matrix[i].cells_y = \
                self.matrix_lengths[i]
            contact_values_matrix.tactile_matrix[i].matrix_id = i
            contact_values_matrix.tactile_matrix[i].tactile_array = \
                contact_values[i]

        if contact_regions and contact_values and centroids_locations:
            status_check = "e_success"

            rospy.loginfo("Publishing largest contact region of each "
                          "phalange...")
            self.pub_contact_regions.publish(largest_contact_regions)

            rospy.loginfo("Publishing contact values of each phalange...")
            self.pub_contact_forces.publish(contact_values_matrix)

            rospy.loginfo("Publishing the force's center of gravity (in which"
                          " cell) for each phalange...")
            self.pub_contact_locations.publish(centroids_locations)
        else:
            status_check = "e_failed"

        self.pub_region_extractor.publish(status_check)

    def raw_tactile_data_cb(self, msg):
        """
        Obtains the tactile information for each phalanx.

        :return: The contact values on each phalanx.
        :rtype: Int[[]]

        :return: The width of the tactile matrix ('cells_x' field).
        :rtype: Int[]

        :return: The length of the tactile matrix ('cells_y' field).
        :rtype: Int[]

        """
        tactile_matrix_length = len(msg.tactile_matrix)
        tactile_info = [[] for _ in range(tactile_matrix_length)]

        matrix_widths = [0] * tactile_matrix_length
        matrix_lengths = [0] * tactile_matrix_length

        for i in range(tactile_matrix_length):
            for j in range(len(msg.tactile_matrix[i].tactile_array)):
                tactile_info[i].append(msg.tactile_matrix[i].tactile_array[j])
            matrix_widths[i] = msg.tactile_matrix[i].cells_x
            matrix_lengths[i] = msg.tactile_matrix[i].cells_y

        self.tactile_info = tactile_info
        self.matrix_widths = matrix_widths
        self.matrix_lengths = matrix_lengths

    def contact_thresholds_cb(self, msg):
        """
        Obtains the contact thresholds for each phalanx,
        according to a 'planner' component.

        :return: The contact thresholds for each phalanx.
        :rtype: Float[]

        """
        self.contact_thresholds = msg.data


class JointMonitors(object):
    """
    Components that monitor the joint position values of an SDH-2.

    """
    def __init__(self):
        # params
        self.current_joint_positions = None
        self.limit_joint_positions = None

        # ToDo: Make these config params
        self.wait_for_data_joint_monitors = 0.2     # seconds
        self.publish_rate_joint_monitors = 0.1      # seconds

        # publishers
        self.pub_joint_monitors_status = rospy.Publisher(
            "e_acquire_joint_monitors_status",
            std_msgs.msg.String, latch=True)
        self.pub_detected_limit_positions = rospy.Publisher(
            "mdr_tactile_grasp_exceeded_positions",
            std_msgs.msg.UInt16MultiArray)

        # subscribers
        rospy.Subscriber("e_limit_positions_detector_start",
                         std_msgs.msg.String,
                         self.start_joint_limit_detector_cb)
        rospy.Subscriber(
            "/sdh_controller/state",
            control_msgs.msg.JointTrajectoryControllerState,
            self.get_current_joint_positions_cb)
        rospy.Subscriber(
            "mdr_tactile_grasp_limit_joint_positions",
            std_msgs.msg.Float32MultiArray, self.get_limit_joint_positions_cb)

    def start_joint_limit_detector_cb(self, msg):
        """
        Starts a 'monitor' component that detects if the limits of the
        joint position values have been exceeded.
        NB: This is a 'dynamic component', i.e. it keeps executing
        until a stopping criteria is met.

        """
        rospy.loginfo("Received '%s' message. Starting joint_limit_detector."
                      % msg.data)
        status_check = "e_failed"

        while not (rospy.is_shutdown() or (self.limit_joint_positions
                                           and self.current_joint_positions)):
            rospy.sleep(self.wait_for_data_joint_monitors)

        stopping_commands = std_msgs.msg.UInt16MultiArray()

        # keeps updating the stopping_commands (since
        # 'self.current_joint_positions' is changing continuously) until all
        # joints are supposed to stop.
        while True:
            stopping_commands.data = joints_computation.detect_limit_positions(
                self.current_joint_positions, self.limit_joint_positions)
            self.pub_detected_limit_positions.publish(stopping_commands)
            rospy.sleep(self.publish_rate_joint_monitors)

            if sum(stopping_commands.data) == 0:
                status_check = "e_success"
                break

        self.pub_joint_monitors_status.publish(status_check)

    def get_current_joint_positions_cb(self, msg):
        """
        Obtains the current joint positions values of each phalanx,
        according to a 'monitor' component.

        :return: The current joint position values for each phalanx.
        :rtype: Float[]

        """
        self.current_joint_positions = msg.actual.positions

    def get_limit_joint_positions_cb(self, msg):
        """
        Obtains the limit joint positions values of each phalanx,
        according to a 'planner' component.

        :return: The limit joint position values for each phalanx.
        :rtype: Float[]

        """
        self.limit_joint_positions = msg.data


def main():
    rospy.init_node("sdh_monitors", anonymous=True)
    tactile_monitors = TactileMonitors()
    joint_monitors = JointMonitors()
    rospy.spin()
