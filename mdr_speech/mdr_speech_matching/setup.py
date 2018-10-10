#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['speech_matching', 'mdr_speech_matching'],
   package_dir={'speech_matching': 'common/src/speech_matching',
                'mdr_speech_matching': 'ros/src/mdr_speech_matching'}
)

setup(**d)
