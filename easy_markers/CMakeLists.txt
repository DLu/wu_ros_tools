cmake_minimum_required(VERSION 3.0.2)
project(easy_markers)
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    interactive_markers
    roslib
    rospy
    std_msgs
    tf
    visualization_msgs
)
catkin_python_setup()
catkin_package(
    CATKIN_DEPENDS geometry_msgs interactive_markers rospy std_msgs tf visualization_msgs
)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslint)
  roslint_python()
  roslint_python(scripts)
  roslint_add_test()
endif()

install(FILES config/interactive.rviz config/std.rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
install(FILES launch/interactive_demo.launch launch/std_demo.launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

catkin_install_python(PROGRAMS scripts/interactive_marker_demo.py scripts/std_marker_demo.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
