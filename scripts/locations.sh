#!/bin/bash
# Room console:  ./scripts/locations.sh            (interactive)
#                ./scripts/locations.sh go "Ward A"
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
require_container
run_in_container rosrun companio_navigation location_cli.py "$@"
