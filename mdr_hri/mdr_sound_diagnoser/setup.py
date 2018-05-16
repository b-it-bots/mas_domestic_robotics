#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mdr_sound_diagnoser'],
    package_dir={'mdr_sound_diagnoser': 'ros/src/mdr_sound_diagnoser'}
)

setup(**d)
