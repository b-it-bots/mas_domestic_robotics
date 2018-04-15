#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['mdr_answer_action'],
   package_dir={'mdr_answer_action': 'ros/src/mdr_answer_action'})

setup(**d)
