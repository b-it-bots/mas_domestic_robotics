#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['mdr_question_matching'],
   package_dir={'mdr_question_matching': 'ros/src/mdr_question_matching'}
)

setup(**d)
