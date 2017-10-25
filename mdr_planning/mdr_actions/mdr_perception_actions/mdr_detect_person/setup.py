#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['mdr_detect_person_action'],
   package_dir={'mdr_detect_person_action': 'src/mdr_detect_person_action'}
)

setup(**d)
