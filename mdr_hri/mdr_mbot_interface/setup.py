#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['mdr_mbot_interface'],
 package_dir={'mdr_mbot_interface': 'ros/src/mdr_mbot_interface'}
)

setup(**d)
