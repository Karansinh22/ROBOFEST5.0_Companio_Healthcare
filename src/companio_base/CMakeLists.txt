cmake_minimum_required(VERSION 3.0.2)
project(companio_base)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs tf
)

catkin_install_python(PROGRAMS
  nodes/motor_driver_node.py
  nodes/cmd_vel_to_wheels.py
  nodes/wheel_odometry.py
  nodes/wheel_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
