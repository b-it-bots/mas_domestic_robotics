#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mdr_demo_simple_pick_and_place'],
    package_dir={'mdr_demo_simple_pick_and_place': 'ros/src/mdr_demo_simple_pick_and_place'}
)

setup(**d)
