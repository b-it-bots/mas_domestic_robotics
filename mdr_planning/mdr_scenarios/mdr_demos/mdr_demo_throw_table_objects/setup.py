#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mdr_demo_throw_table_objects'],
    package_dir={'mdr_demo_throw_table_objects': 'ros/src/mdr_demo_throw_table_objects'}
)

setup(**d)
