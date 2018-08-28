#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mdr_component_monitoring'],
    package_dir={'mdr_component_monitoring': 'src/mdr_component_monitoring'}
)

setup(**d)
