#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['mdr_speech_person_recognition'],
   package_dir={'mdr_speech_person_recognition': 'ros/src/mdr_speech_person_recognition'}
)

setup(**d)
