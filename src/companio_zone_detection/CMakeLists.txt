cmake_minimum_required(VERSION 3.0.2)
project(companio_zone_detection)
find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs cv_bridge std_msgs)
catkin_package(CATKIN_DEPENDS rospy sensor_msgs cv_bridge std_msgs)
catkin_install_python(PROGRAMS
  scripts/zone_detector.py
  scripts/camera_publisher.py
  scripts/find_camera.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS scripts/test_camera.sh scripts/diagnose_camera.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
