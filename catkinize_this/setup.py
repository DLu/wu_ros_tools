from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = []
d['scripts'] = ['src/catkinize_this.py']
d['package_dir'] = {}

setup(**d)
