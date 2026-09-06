#!/bin/bash
# Hardware bring-up + keyboard teleop (i/j/k/l/, in this terminal).
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
require_container
echo "Keyboard: i forward | , back | j left | l right | k stop | q/z speed +/-"
run_in_container roslaunch companio_bringup teleop.launch "$@"
