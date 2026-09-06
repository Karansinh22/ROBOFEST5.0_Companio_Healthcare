#!/bin/bash
# Build the Companio ROS Noetic image (first run only), start the container with
# every hardware device mapped, and build the catkin workspace inside it.
#
#   ./scripts/setup_container.sh            # start (or reuse) the container and build
#   ./scripts/setup_container.sh --rebuild  # recreate the container and rebuild the image
set -e
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

REBUILD=false
[ "$1" == "--rebuild" ] && REBUILD=true

echo "=========================================="
echo " Companio container setup"
echo " workspace: $WORKSPACE_DIR"
echo "=========================================="

if container_running && [ "$REBUILD" = false ]; then
    echo "[1/5] Container '$CONTAINER_NAME' already running - skipping creation."
else
    echo "[1/5] Removing old container ..."
    docker stop "$CONTAINER_NAME" 2>/dev/null || true
    docker rm "$CONTAINER_NAME" 2>/dev/null || true
    if [ "$REBUILD" = true ]; then
        docker rmi "$IMAGE_NAME" 2>/dev/null || true
    fi

    echo "[2/5] Checking hardware ..."
    ls -l /dev/motor_left /dev/motor_right /dev/rplidar 2>/dev/null \
        || echo "  warning: /dev/motor_* or /dev/rplidar missing - run: sudo ./scripts/install_udev_rules.sh"
    lsusb | grep -qi realsense && echo "  RealSense camera detected" || echo "  warning: RealSense camera not detected"
    [ -e /dev/i2c-1 ] && echo "  I2C bus present (arms)" || echo "  warning: /dev/i2c-1 missing - enable I2C in raspi-config"

    echo "[3/5] Docker image ..."
    if ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
        echo "  building $IMAGE_NAME (this takes a while on the first run)"
        docker build -t "$IMAGE_NAME" -f "$WORKSPACE_DIR/docker/Dockerfile" "$WORKSPACE_DIR/docker"
    else
        echo "  image found"
    fi

    echo "[4/5] Starting container ..."
    if command -v xhost >/dev/null 2>&1; then xhost +local:docker >/dev/null 2>&1 || true; fi
    touch "$HOME/.Xauthority"

    DEVICE_ARGS=""
    for dev in /dev/i2c-* /dev/dri/* /dev/hailo0 /dev/gpiochip* /dev/gpiomem /dev/snd; do
        [ -e "$dev" ] && DEVICE_ARGS="$DEVICE_ARGS --device=$dev"
    done
    I2C_GID=$(getent group i2c 2>/dev/null | cut -d: -f3)
    [ -n "$I2C_GID" ] && DEVICE_ARGS="$DEVICE_ARGS --group-add $I2C_GID"

    docker run -d \
        --name "$CONTAINER_NAME" \
        --net=host --ipc=host --privileged \
        --restart unless-stopped \
        --add-host="$(hostname):127.0.0.1" \
        -e DISPLAY="${DISPLAY:-:0}" \
        -e QT_X11_NO_MITSHM=1 \
        -e XAUTHORITY=/root/.Xauthority \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v "$HOME/.Xauthority:/root/.Xauthority:rw" \
        -v /dev:/dev -v /sys:/sys -v /run/udev:/run/udev:ro \
        -v "$WORKSPACE_DIR:$CONTAINER_WS" \
        --group-add dialout \
        $DEVICE_ARGS \
        "$IMAGE_NAME" \
        bash -c "tail -f /dev/null"
    echo "  container started"
fi

echo "[5/5] Building workspace ..."
docker exec "$CONTAINER_NAME" bash -c "
    set -e
    source /opt/ros/noetic/setup.bash
    cd $CONTAINER_WS
    find src -name '*.py' -exec chmod +x {} \;
    catkin_make -j\$(nproc) || catkin_make -j2
    source devel/setup.bash
    echo; echo 'Packages:'; rospack list | grep companio_ | awk '{print \"  \" \$1}'
"

echo
echo "Done. Next steps:"
echo "  ./scripts/start_teleop.sh       drive with the keyboard"
echo "  ./scripts/start_mapping.sh      build a map and save rooms"
echo "  ./scripts/start_navigation.sh   autonomous navigation + web dashboard"
echo "  ./scripts/enter_container.sh    open a shell inside the container"
