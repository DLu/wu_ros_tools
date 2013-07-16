from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

package_info = genderate_distutils_setup(
    packages=['easy_markers'],
    package_dir={'easy_markers': 'src'},
    requires=['rospy'])
)

setup(**setup_args)

