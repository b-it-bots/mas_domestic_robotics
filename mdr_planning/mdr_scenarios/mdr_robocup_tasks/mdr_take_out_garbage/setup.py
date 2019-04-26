#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mdr_take_out_garbage'],
    package_dir={'mdr_take_out_garbage': 'ros/src/mdr_take_out_garbage'}
)

setup(**d)
