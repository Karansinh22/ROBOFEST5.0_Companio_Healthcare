#!/bin/bash
# Open RViz (needs a display on the Pi or X forwarding):  ./scripts/start_rviz.sh [mapping|navigation]
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
require_container
run_in_container roslaunch companio_description rviz.launch view:="${1:-navigation}"
