#!/bin/bash
# Run any ROS command inside the container, e.g.
#   ./scripts/run.sh rosrun companio_navigation location_cli.py
#   ./scripts/run.sh roslaunch companio_arms arms.launch
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
require_container
run_in_container "$@"
