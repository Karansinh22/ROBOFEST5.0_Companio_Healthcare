#!/bin/bash
# Open an interactive shell inside the running Companio container (ROS sourced).
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
require_container
run_in_container bash
