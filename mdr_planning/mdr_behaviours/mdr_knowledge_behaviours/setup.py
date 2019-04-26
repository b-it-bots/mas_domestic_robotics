#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mdr_knowledge_behaviours'],
    package_dir={'mdr_knowledge_behaviours': 'ros/src/mdr_knowledge_behaviours'}
)

setup(**d)
