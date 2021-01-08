#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mdr_wrs_tidy_up'],
    package_dir={'mdr_wrs_tidy_up': 'ros/src/mdr_wrs_tidy_up'}
)

setup(**d)
