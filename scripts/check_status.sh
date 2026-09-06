#!/bin/bash
# Quick health check of the host, the container and the robot's devices.
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
echo "== Container =="
if container_running; then echo "running: $CONTAINER_NAME"; else echo "NOT running (./scripts/setup_container.sh)"; fi
echo; echo "== Serial devices (host) =="
ls -l /dev/rplidar /dev/motor_left /dev/motor_right 2>/dev/null || echo "symlinks missing - sudo ./scripts/install_udev_rules.sh"
ls /dev/ttyUSB* 2>/dev/null || echo "no ttyUSB devices"
echo; echo "== Other hardware =="
lsusb | grep -i realsense || echo "RealSense not detected"
ls /dev/i2c-* 2>/dev/null || echo "no I2C bus (arms)"
[ -e /dev/hailo0 ] && echo "Hailo AI HAT present" || echo "Hailo AI HAT not present (optional)"
arecord -l 2>/dev/null | grep -i card || echo "no microphone (voice control optional)"
if container_running; then
    echo; echo "== Workspace (container) =="
    docker exec "$CONTAINER_NAME" bash -c "source /opt/ros/noetic/setup.bash; [ -f $CONTAINER_WS/devel/setup.bash ] && source $CONTAINER_WS/devel/setup.bash && rospack list | grep companio_ || echo 'workspace not built'"
    echo; echo "== ROS =="
    docker exec "$CONTAINER_NAME" bash -c "source /opt/ros/noetic/setup.bash; rostopic list 2>/dev/null | head -20 || echo 'roscore not running'"
fi
