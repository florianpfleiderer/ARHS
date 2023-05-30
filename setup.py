# from distutils.core import setup
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['player', 'data_utils', 'field_components', 'globals', 'list_utils', 'math_utils', 'player_utils', 'visualization', 'ref_com'],
    scripts=['scripts'],
    package_dir={'': 'src'}
)

setup(**setup_args)
