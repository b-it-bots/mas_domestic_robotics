#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['mdr_recognize_people_action'],
   package_dir={'mdr_recognize_people_action': 'ros/src/mdr_recognize_people_action'}
)

setup(**d)
