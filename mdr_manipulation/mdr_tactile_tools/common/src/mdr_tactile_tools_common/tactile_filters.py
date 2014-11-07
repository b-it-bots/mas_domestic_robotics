#!/usr/bin/env python
"""
This module contains filters to process the information of the
tactile sensors on a Schunk Dexterous Hand (SDH-2).


"""
#-*- encoding: utf-8 -*-
__author__ = 'jsanch'

import numpy


def detect_contact(tactile_sensors, thresholds):
    """
    Detects which tactile sensors have a contact beyond a mean threshold value.

    Args:
        tactile_sensors: Int[[]]
            The pressure values on the tactile sensors.
        thresholds: Int[]
            The minimum mean pressure value for each tactile sensor to be regarded
            as a contact.

    Returns:
        contact_list: Int[]
            Phalanges that have a contact greater or equal to their threshold
            value have a value of '1', a value of '0' is assigned otherwise.

    """
    contact_list = []

    for i, sensor in enumerate(tactile_sensors):
        if numpy.mean(sensor) >= thresholds[i]:
            contact_list.append(1)
        else:
            contact_list.append(0)

    return contact_list
