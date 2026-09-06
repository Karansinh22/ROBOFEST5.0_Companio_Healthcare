cmake_minimum_required(VERSION 3.0.2)
project(companio_voice)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs geometry_msgs companio_msgs)
catkin_package(CATKIN_DEPENDS rospy std_msgs geometry_msgs companio_msgs)
catkin_install_python(PROGRAMS
  scripts/voice_teleop.py
  scripts/voice_navigator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
