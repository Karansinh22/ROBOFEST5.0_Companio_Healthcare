cmake_minimum_required(VERSION 3.0.2)
project(companio_bringup)
find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs)
catkin_package()
catkin_install_python(PROGRAMS scripts/simple_camera_viewer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
