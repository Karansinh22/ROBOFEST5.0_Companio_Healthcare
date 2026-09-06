cmake_minimum_required(VERSION 3.0.2)
project(companio_description)
find_package(catkin REQUIRED)
catkin_package()
install(DIRECTORY urdf rviz maps launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
