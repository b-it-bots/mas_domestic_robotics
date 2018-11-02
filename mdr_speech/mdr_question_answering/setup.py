#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['mdr_question_answering', 'ip_info', 'weather_api'],
   package_dir={'mdr_question_answering': 'ros/src/mdr_question_answering',
                'ip_info': 'common/src/ip_info',
                'weather_api': 'common/src/weather_api'}
)

setup(**d)
