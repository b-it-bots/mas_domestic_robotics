#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mdr_recognize_gesture_action', 'mdr_recognize_gesture'],
    package_dir={'mdr_recognize_gesture_action': 'ros/src/mdr_recognize_gesture_action',
                 'mdr_recognize_gesture': 'common/mdr_recognize_gesture'}
)

setup(**d)
