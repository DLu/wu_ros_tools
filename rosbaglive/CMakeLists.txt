cmake_minimum_required(VERSION 3.0.2)
project(rosbaglive)
find_package(catkin REQUIRED COMPONENTS rosbag rospy)
catkin_package(
    CATKIN_DEPENDS rosbag rospy
)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslint)
  roslint_python(scripts)
  roslint_add_test()
endif()

catkin_install_python(PROGRAMS scripts/play.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
