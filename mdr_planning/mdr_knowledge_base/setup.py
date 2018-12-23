#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['knowledge_utils'],
    package_dir={'knowledge_utils': 'common/knowledge_utils'}
)

setup(**d)
