#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['mdr_introduce_self_action'],
   package_dir={'mdr_introduce_self_action': 'ros/src/mdr_introduce_self_action'}
)

setup(**d)
