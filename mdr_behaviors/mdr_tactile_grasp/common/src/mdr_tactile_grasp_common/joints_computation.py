#!/usr/bin/env python
"""
This module contains functions to compute required information for
joint values (e.g. positions, velocities, accelerations).

"""
#-*- encoding: utf-8 -*-
__author__ = 'jsanch'

ROUND_DIGITS = 4


def detect_limit_positions(current_positions, limit_positions):
    """
    Detects when a joint position of a phalange has reached its limit joint position and
    it sets that phalange to a '0' (i.e. to stop its movement), otherwise it sets it to
    '1' (i.e. to keep it moving).

    Args:
        current_positions: Float[]
            Specifies the current positions.
        limit_positions: Float[]
            Specifies the threshold value of positions that should not be exceeded.

    Returns:
        positions: Int[]
            Specifies which positions have exceeded their threshold, it assigns a '0'
            when there is a limit position has been reached, otherwise it assigns a '1'.

    """
    rounded_positions = [round(position, ROUND_DIGITS) for
                         position in current_positions]

    stop_criteria = [1 if x < limit_positions[i] else 0
                     for i, x in enumerate(rounded_positions)]

    return stop_criteria
