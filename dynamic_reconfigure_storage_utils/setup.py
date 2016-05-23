#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['dynamic_reconfigure_storage_utils'],
    package_dir={'': 'src'},
    requires=['pyyaml']
)

setup(**d)
