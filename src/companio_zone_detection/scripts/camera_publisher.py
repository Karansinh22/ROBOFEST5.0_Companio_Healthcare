#!/usr/bin/env python3
"""
Simple ROS camera publisher node using OpenCV.
Publishes camera images to /camera/image_raw topic.
"""

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class CameraPublisher:
    def __init__(self):
        rospy.init_node("camera_publisher", anonymous=True)
        
        self.bridge = CvBridge()
        
        # Get camera device parameter (default to /dev/video20)
        # Try multiple devices if specified one doesn't work
        self.camera_device = rospy.get_param("~camera_device", "/dev/video20")
        self.frame_width = rospy.get_param("~width", 640)
        self.frame_height = rospy.get_param("~height", 480)
        self.fps = rospy.get_param("~fps", 30)
        
        # Publisher
        self.image_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=1)
        
        # Initialize camera - try multiple devices if needed
        devices_to_try = [self.camera_device]
        # Also try common video device numbers
        if "/dev/video" in self.camera_device:
            try:
                base_num = int(self.camera_device.replace("/dev/video", ""))
                # Try nearby devices
                for offset in [-2, -1, 1, 2, 3, 4, 5]:
                    devices_to_try.append(f"/dev/video{base_num + offset}")
            except:
                pass
        
        self.cap = None
        for device in devices_to_try:
            rospy.loginfo("Trying camera device: %s", device)
            # Use V4L2 backend explicitly (important for Raspberry Pi)
            cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
            if cap.isOpened():
                # Test if we can actually read a frame
                ret, frame = cap.read()
                if ret and frame is not None:
                    self.cap = cap
                    self.camera_device = device
                    rospy.loginfo("Successfully opened camera: %s", device)
                    break
                else:
                    cap.release()
            else:
                if cap:
                    cap.release()
        
        if self.cap is None or not self.cap.isOpened():
            rospy.logerr("Failed to open any camera device")
            rospy.logerr("Tried devices: %s", devices_to_try)
            rospy.logerr("Trying to list available video devices...")
            import subprocess
            try:
                result = subprocess.run(['v4l2-ctl', '--list-devices'], 
                                      capture_output=True, text=True, timeout=2)
                rospy.logerr("Available devices:\n%s", result.stdout)
            except:
                rospy.logerr("Could not list devices. Please check camera connection.")
            raise RuntimeError("Camera initialization failed")
        
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        # Get actual properties
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = int(self.cap.get(cv2.CAP_PROP_FPS))
        
        rospy.loginfo("Camera opened successfully!")
        rospy.loginfo("Resolution: %dx%d @ %d fps", actual_width, actual_height, actual_fps)
        
        # Rate limiter
        self.rate = rospy.Rate(self.fps)
        
    def run(self):
        rospy.loginfo("Publishing camera images to /camera/image_raw")
        
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            
            if not ret:
                rospy.logwarn("Failed to read frame from camera")
                self.rate.sleep()
                continue
            
            try:
                # Convert OpenCV image to ROS Image message
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                ros_image.header.stamp = rospy.Time.now()
                ros_image.header.frame_id = "camera_frame"
                
                # Publish
                self.image_pub.publish(ros_image)
                
            except CvBridgeError as e:
                rospy.logerr("CvBridge Error: %s", e)
            
            self.rate.sleep()
    
    def shutdown(self):
        rospy.loginfo("Shutting down camera publisher...")
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        camera = CameraPublisher()
        camera.run()
    except rospy.ROSInterruptException:
        pass
    except RuntimeError as e:
        rospy.logerr("Camera publisher failed: %s", e)
    finally:
        if 'camera' in locals():
            camera.shutdown()

