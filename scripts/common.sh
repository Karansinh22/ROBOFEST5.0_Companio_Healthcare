#!/bin/bash
# Shared helpers for the Companio host-side scripts (sourced, not executed).

CONTAINER_NAME="companio"
IMAGE_NAME="companio/ros-noetic:latest"
CONTAINER_WS="/root/companio_ws"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

container_running() {
    [ -n "$(docker ps -q -f "name=^${CONTAINER_NAME}$")" ]
}

require_container() {
    if ! container_running; then
        echo "Container '$CONTAINER_NAME' is not running. Start it with: ./scripts/setup_container.sh"
        exit 1
    fi
}

# run_in_container "<command>"  - interactive shell inside the container with ROS sourced
run_in_container() {
    docker exec -it \
        -e DISPLAY="${DISPLAY:-:0}" \
        "$CONTAINER_NAME" bash -lc "source /opt/ros/noetic/setup.bash; [ -f $CONTAINER_WS/devel/setup.bash ] && source $CONTAINER_WS/devel/setup.bash; cd $CONTAINER_WS; $*"
}
