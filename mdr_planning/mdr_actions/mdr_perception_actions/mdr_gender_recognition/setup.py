#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['mdr_gender_recognition'],
   package_dir={'mdr_gender_recognition': 'ros/src/mdr_gender_recognition'}
)

setup(**d)
