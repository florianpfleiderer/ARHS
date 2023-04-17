# from distutils.core import setup
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['player', 'player_utils', 'math_utils', 'globals', 'field_components'],
    scripts=['scripts'],
    package_dir={'': 'src'}
)

setup(**setup_args)
