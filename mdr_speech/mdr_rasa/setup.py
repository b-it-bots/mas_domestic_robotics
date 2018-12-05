#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mdr_rasa_nlu_wrapper'],
    package_dir={'mdr_rasa_nlu_wrapper': 'ros/src/mdr_rasa_nlu_wrapper'}
)

setup(**d)
