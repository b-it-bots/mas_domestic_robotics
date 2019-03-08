#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mdr_serve_drinks'],
    package_dir={'mdr_serve_drinks': 'ros/src/mdr_serve_drinks'}
)

setup(**d)
