cmake_minimum_required(VERSION 2.8.3)
project(catkinize_this)
find_package(catkin)
install(DIRECTORY scripts/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
catkin_package()
