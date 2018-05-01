#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mdr_store_groceries'],
    package_dir={'mdr_store_groceries': 'ros/src/mdr_store_groceries'}
)

setup(**d)
