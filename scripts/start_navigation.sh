#!/bin/bash
# Autonomous navigation on a saved map + RViz + web dashboard (http://<robot-ip>:8000).
#   ./scripts/start_navigation.sh
#   ./scripts/start_navigation.sh map_name:=ward_b rviz:=false
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
require_container
IP=$(hostname -I 2>/dev/null | awk '{print $1}')
echo "Dashboard: http://${IP:-<robot-ip>}:8000    Give AMCL an initial pose before the first goal."
run_in_container roslaunch companio_bringup navigation.launch "$@"
