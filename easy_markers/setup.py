from distutils.core import setup
from catkin_pkg.package import parse_package_for_distutils


package_info = parse_package_for_distutils()
package_info['packages'] = ['easy_markers']
package_info['package_dir'] = {'easy_markers': 'src'}
package_info['install_requires'] = []

setup(**package_info)
