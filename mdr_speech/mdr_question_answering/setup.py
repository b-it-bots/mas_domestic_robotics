#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['mdr_question_answering'],
   package_dir={'mdr_question_answering': 'ros/src/mdr_question_answering'}
)

setup(**d)
