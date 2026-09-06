cmake_minimum_required(VERSION 3.0.2)
project(companio_arms)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs sensor_msgs)
catkin_package(CATKIN_DEPENDS rospy std_msgs sensor_msgs)

catkin_install_python(PROGRAMS
  scripts/left_arm_controller.py
  scripts/right_arm_controller.py
  scripts/left_arm_ik_controller.py
  scripts/right_arm_ik_controller.py
  scripts/right_arm_numerical_ik.py
  scripts/left_arm_sequence.py
  scripts/right_arm_sequence.py
  scripts/arm_teleop.py
  scripts/dual_arm_teleop.py
  scripts/autonomous_arm_teleop.py
  scripts/autonomous_left_arm_teleop.py
  scripts/tower_controller.py
  scripts/tower_teleop.py
  scripts/gripper_vision_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
