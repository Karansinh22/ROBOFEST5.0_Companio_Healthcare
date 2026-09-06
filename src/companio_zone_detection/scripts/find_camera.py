#!/usr/bin/env python3
"""
Script to find and test camera devices.
Tests all available /dev/video* devices to find which one is a working camera.
"""

import cv2
import glob
import sys

def test_camera_device(device_path):
    """Test if a video device can be opened and read from."""
    try:
        # Use V4L2 backend explicitly (important for Raspberry Pi)
        cap = cv2.VideoCapture(device_path, cv2.CAP_V4L2)
        if not cap.isOpened():
            return False, None, None
        
        # Try to read a frame
        ret, frame = cap.read()
        if not ret or frame is None:
            cap.release()
            return False, None, None
        
        # Get device properties
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = int(cap.get(cv2.CAP_PROP_FPS))
        
        cap.release()
        return True, (width, height), fps
        
    except Exception as e:
        return False, None, None

def main():
    print("=" * 70)
    print("CAMERA DEVICE FINDER")
    print("=" * 70)
    print("\nScanning for video devices...\n")
    
    # Find all video devices
    video_devices = sorted(glob.glob("/dev/video*"))
    
    if not video_devices:
        print("❌ No video devices found in /dev/video*")
        print("\nPlease check:")
        print("  1. Camera is connected via USB")
        print("  2. Camera drivers are installed")
        print("  3. You have permissions to access /dev/video*")
        return 1
    
    print(f"Found {len(video_devices)} video device(s):\n")
    
    working_cameras = []
    
    for device in video_devices:
        print(f"Testing {device}...", end=" ", flush=True)
        works, resolution, fps = test_camera_device(device)
        
        if works:
            print(f"✅ WORKS!")
            print(f"   Resolution: {resolution[0]}x{resolution[1]}")
            print(f"   FPS: {fps}")
            working_cameras.append((device, resolution, fps))
        else:
            print("❌ Cannot read frames")
        print()
    
    print("=" * 70)
    if working_cameras:
        print(f"\n✅ Found {len(working_cameras)} working camera(s):\n")
        for device, resolution, fps in working_cameras:
            print(f"  📷 {device}")
            print(f"     Resolution: {resolution[0]}x{resolution[1]} @ {fps} fps")
            print(f"\n  Use this device with:")
            print(f"     rosrun companio_zone_detection camera_publisher.py _camera_device:={device}")
            print(f"  Or in launch file:")
            print(f"     <param name=\"camera_device\" value=\"{device}\" />")
            print()
    else:
        print("\n❌ No working cameras found!")
        print("\nTroubleshooting:")
        print("  1. Check camera is connected: lsusb | grep -i camera")
        print("  2. Check video devices: ls -la /dev/video*")
        print("  3. Try running with sudo (if permission issue)")
        print("  4. Check if camera is being used by another process")
        return 1
    
    print("=" * 70)
    return 0

if __name__ == "__main__":
    sys.exit(main())

