#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['mdr_perceive_table'],
   package_dir={'mdr_perceive_table': 'ros/src/mdr_perceive_table'}
)

setup(**d)
