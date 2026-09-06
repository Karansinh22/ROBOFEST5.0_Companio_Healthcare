#!/bin/bash
# Comprehensive camera diagnostic script

echo "=========================================="
echo "CAMERA DIAGNOSTIC TOOL"
echo "=========================================="
echo ""

echo "1. Checking USB devices..."
echo "----------------------------"
if command -v lsusb &> /dev/null; then
    lsusb | grep -i -E 'camera|video|imaging|webcam' || echo "   No camera found in USB devices"
    echo ""
    echo "   All USB devices:"
    lsusb | head -10
else
    echo "   lsusb not available (run on host, not in container)"
fi
echo ""

echo "2. Video devices found:"
echo "----------------------------"
v4l2-ctl --list-devices 2>/dev/null || echo "   v4l2-ctl not available"
echo ""

echo "3. Checking /dev/video* devices:"
echo "----------------------------"
ls -la /dev/video* 2>/dev/null | head -10 || echo "   No video devices found"
echo ""

echo "4. Testing video devices (this may take a moment)..."
echo "----------------------------"
working_found=false
for dev in /dev/video{0..35}; do
    if [ -e "$dev" ]; then
        # Try to get device info
        info=$(v4l2-ctl -d "$dev" --info 2>&1 | head -1)
        if [ $? -eq 0 ] && [ ! -z "$info" ]; then
            echo "   $dev: $info"
            # Try to read a frame with Python
            python3 -c "
import cv2
cap = cv2.VideoCapture('$dev', cv2.CAP_V4L2)
if cap.isOpened():
    ret, frame = cap.read()
    if ret and frame is not None:
        print('      ✅ CAN READ FRAMES - This is a working camera!')
        print('      Resolution: {}x{}'.format(frame.shape[1], frame.shape[0]))
    else:
        print('      ⚠️  Opens but cannot read (may be internal Pi device)')
    cap.release()
else:
    print('      ❌ Cannot open')
" 2>/dev/null && working_found=true
        fi
    fi
done

echo ""
echo "=========================================="
if [ "$working_found" = false ]; then
    echo "❌ NO WORKING USB CAMERA FOUND"
    echo ""
    echo "Possible reasons:"
    echo "  1. Camera is not connected"
    echo "  2. Camera is connected but drivers not loaded"
    echo "  3. Camera needs to be accessed from host (not container)"
    echo "  4. Camera is on a different device (check dmesg)"
    echo ""
    echo "Next steps:"
    echo "  1. Connect USB camera to Raspberry Pi"
    echo "  2. Check: dmesg | tail -20 (look for camera detection)"
    echo "  3. If camera is on host, you may need to:"
    echo "     - Pass /dev/video* device to container with --device flag"
    echo "     - Or run camera node on host instead of container"
    echo ""
    echo "To pass video device to container, add to docker run:"
    echo "  --device=/dev/videoX:/dev/videoX"
else
    echo "✅ Camera found! Use the device path shown above."
fi
echo "=========================================="

