cmake_minimum_required(VERSION 3.0.2)
project(companio_navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  visualization_msgs
  move_base_msgs
  actionlib
  tf
  interactive_markers
  companio_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs std_srvs geometry_msgs nav_msgs visualization_msgs
                 move_base_msgs actionlib tf interactive_markers companio_msgs
)

catkin_install_python(PROGRAMS
  scripts/location_manager.py
  scripts/location_cli.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
