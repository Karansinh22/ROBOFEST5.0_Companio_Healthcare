#!/bin/bash
# Rebuild the catkin workspace inside the container (after editing code).
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
require_container
run_in_container "find src -name '*.py' -exec chmod +x {} \; && catkin_make -j\$(nproc)"
