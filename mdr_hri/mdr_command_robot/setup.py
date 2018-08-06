#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mdr_command_robot'],
    package_dir={'mdr_command_robot': 'ros/src/mdr_command_robot'}
)

setup(**d)
