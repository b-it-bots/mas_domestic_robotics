from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['hsr_task_sm', 'hsr_task_sm.states'],
    package_dir={'': 'ros/src'}
)

setup(**d)
