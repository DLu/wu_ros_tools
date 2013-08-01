cmake_minimum_required(VERSION 2.8.3)
project(easy_markers)
find_package(catkin)
install(DIRECTORY scripts/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_python_setup()
catkin_package()
