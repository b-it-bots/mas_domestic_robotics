#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['mdr_listen_action_ros'],
   package_dir={'mdr_listen_action_ros': 'ros/src/mdr_listen_action_ros'})

setup(**d)
