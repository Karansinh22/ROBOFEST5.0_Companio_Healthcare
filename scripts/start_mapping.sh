#!/bin/bash
# SLAM mapping with RViz and the web dashboard.  Drive with the keyboard in this
# terminal; save rooms from RViz (Publish Point), the dashboard, or the CLI.
#   ./scripts/start_mapping.sh                      # map_name=hospital_map
#   ./scripts/start_mapping.sh map_name:=ward_b rviz:=false
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
require_container
echo "When the map looks complete, run in another terminal:  ./scripts/save_map.sh <map_name>"
run_in_container roslaunch companio_bringup mapping.launch "$@"
