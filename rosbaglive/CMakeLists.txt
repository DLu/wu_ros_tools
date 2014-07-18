cmake_minimum_required(VERSION 2.8.3)
project(rosbaglive)
find_package(catkin REQUIRED COMPONENTS rospy rosbag)
catkin_package()
install(DIRECTORY scripts/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

