#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['mdr_base_planner'],
   package_dir={'mdr_base_planner': 'ros/src/mdr_base_planner'}
)

setup(**d)
