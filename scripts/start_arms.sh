#!/bin/bash
# Arm + tower controllers (PCA9685 over I2C).  Then use:
#   ./scripts/run.sh rosrun companio_arms arm_teleop.py
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
require_container
run_in_container roslaunch companio_arms arms.launch "$@"
