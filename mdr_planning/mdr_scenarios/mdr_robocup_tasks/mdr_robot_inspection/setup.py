#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['mdr_robot_inspection'],
   package_dir={'mdr_robot_inspection': 'src/mdr_robot_inspection'}
)

setup(**d)
