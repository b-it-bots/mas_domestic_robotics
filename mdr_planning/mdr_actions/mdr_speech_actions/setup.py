#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(\
   packages=['listen_action', 'answer_action', 'ask_action'],\
   package_dir={'listen_action':'src/listen_action', 'answer_action':'src/answer_action',\
                'ask_action':'src/ask_action'})

setup(**d)