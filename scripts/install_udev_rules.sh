#!/bin/bash
# Install the Companio udev rules on the Raspberry Pi host (run with sudo).
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ "$EUID" -ne 0 ]; then echo "Run with sudo: sudo $0"; exit 1; fi
cp "$SCRIPT_DIR/../config/udev/99-companio.rules" /etc/udev/rules.d/99-companio.rules
udevadm control --reload-rules && udevadm trigger
sleep 1
echo "Installed. Devices:"; ls -l /dev/rplidar /dev/motor_left /dev/motor_right 2>/dev/null || echo "  (plug in the USB hub and check again; use scripts/identify_motors.py to confirm ports)"
