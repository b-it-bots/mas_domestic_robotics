#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mdr_receive_object_action'],
    package_dir={'mdr_receive_object_action': 'ros/src/mdr_receive_object_action'}
)

setup(**d)
