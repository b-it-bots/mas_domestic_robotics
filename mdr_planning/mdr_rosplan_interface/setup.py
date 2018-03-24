#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['mdr_rosplan_interface'],
   package_dir={'mdr_rosplan_interface': 'ros/src/mdr_rosplan_interface'}
)

setup(**d)
