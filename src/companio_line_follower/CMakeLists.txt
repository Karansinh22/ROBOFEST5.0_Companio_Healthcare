cmake_minimum_required(VERSION 3.0.2)
project(companio_line_follower)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs sensor_msgs geometry_msgs)
catkin_package(CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs)

catkin_install_python(PROGRAMS
  scripts/line_following.py
  scripts/line_following_black.py
  scripts/line_path_following.py
  scripts/keyboard_line_following.py
  scripts/voice_line_following.py
  scripts/line_follower_udp.py
  scripts/ai_line_follower_direct.py
  scripts/ai_destination_follower.py
  scripts/path_following.py
  scripts/hailo_line_detector_node.py
  scripts/line_follower_node.py
  scripts/line_following_node.py
  scripts/motor_control_node.py
  scripts/npu_inference_node.py
  scripts/camera_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
