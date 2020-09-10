#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mdr_head_controller'],
    package_dir={'mdr_head_controller': 'ros/src/mdr_head_controller'}
)

setup(**d)
