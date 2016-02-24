#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['mdr_haptic'],
   package_dir={'mdr_haptic': 'ros/src/mdr_haptic'}
)

setup(**d)
