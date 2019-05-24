#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mdr_planning_behaviours'],
    package_dir={'mdr_planning_behaviours': 'ros/src/mdr_planning_behaviours'}
)

setup(**d)
