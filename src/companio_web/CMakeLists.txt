cmake_minimum_required(VERSION 3.0.2)
project(companio_web)
find_package(catkin REQUIRED COMPONENTS rospy)
catkin_package()
catkin_install_python(PROGRAMS scripts/web_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY www launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
