#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['knowledge_utils', 'mdr_knowledge_base'],
    package_dir={'knowledge_utils': 'common/knowledge_utils',
                 'mdr_knowledge_base': 'ros/src/mdr_knowledge_base'}
)

setup(**d)
