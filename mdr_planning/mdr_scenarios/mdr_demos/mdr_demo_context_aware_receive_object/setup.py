#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mdr_demo_context_aware_receive_object'],
    package_dir={'mdr_demo_context_aware_receive_object': 'ros/src/mdr_demo_context_aware_receive_object'}
)

setup(**d)
