#!/bin/bash
# Quick test script to find and test camera device

echo "=========================================="
echo "Testing Camera Devices"
echo "=========================================="

# Try to find a working camera device
for device in /dev/video{0..35}; do
    if [ -e "$device" ]; then
        echo "Testing $device..."
        timeout 2 python3 -c "
import cv2
cap = cv2.VideoCapture('$device')
if cap.isOpened():
    ret, frame = cap.read()
    if ret and frame is not None:
        print('  ✓ $device WORKS! (Resolution: {}x{})'.format(frame.shape[1], frame.shape[0]))
        cap.release()
        exit(0)
    else:
        print('  ✗ $device opened but cannot read frames')
        cap.release()
        exit(1)
else:
    print('  ✗ $device cannot be opened')
    exit(1)
" 2>/dev/null
        if [ $? -eq 0 ]; then
            echo ""
            echo "Found working camera: $device"
            echo "You can use this device in the launch file or with:"
            echo "  rosrun companio_zone_detection camera_publisher.py _camera_device:=$device"
            exit 0
        fi
    fi
done

echo ""
echo "No working camera device found!"
echo "Please check:"
echo "  1. Camera is connected via USB"
echo "  2. Camera drivers are installed"
echo "  3. You have permissions to access /dev/video*"

