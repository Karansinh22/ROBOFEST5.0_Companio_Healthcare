#!/usr/bin/env python3
"""
ROS Noetic node for detecting safe (blue) paths and infection (red) zones using OpenCV.

This node:
- Subscribes to camera image topic (/camera/image_raw)
- Converts images to HSV color space for robust color detection
- Segments blue (safe) and red (infection) zones
- Applies morphological operations for noise reduction
- Publishes detection masks for navigation logic
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool


class ZoneDetector:
    def __init__(self):
        rospy.init_node("zone_detector", anonymous=True)

        self.bridge = CvBridge()
        
        # Subscribe to camera image
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_cb)
        
        # Publishers for masks (optional - can be used by navigation nodes)
        self.safe_mask_pub = rospy.Publisher("/safe_mask", Image, queue_size=1)
        self.infection_mask_pub = rospy.Publisher("/infection_mask", Image, queue_size=1)
        
        # Publisher for detection status (optional)
        self.safe_detected_pub = rospy.Publisher("/safe_detected", Bool, queue_size=1)
        self.infection_detected_pub = rospy.Publisher("/infection_detected", Bool, queue_size=1)

        # Debug mode - show OpenCV windows
        self.debug = rospy.get_param("~debug", True)
        
        # Detection thresholds (minimum pixels to consider as detected)
        self.safe_threshold = rospy.get_param("~safe_threshold", 100)
        self.infection_threshold = rospy.get_param("~infection_threshold", 100)

        # Color ranges in HSV
        # Blue (Safe Path)
        self.low_blue = np.array([100, 120, 70])
        self.high_blue = np.array([140, 255, 255])
        
        # Red (Infection Zone) - Red wraps around in HSV, need 2 ranges
        self.low_red1 = np.array([0, 120, 70])
        self.high_red1 = np.array([10, 255, 255])
        self.low_red2 = np.array([170, 120, 70])
        self.high_red2 = np.array([180, 255, 255])

        # Morphology kernel for noise reduction
        self.kernel_size = rospy.get_param("~kernel_size", 5)
        self.kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)

        # Rate limiter for terminal output (print every N frames)
        self.frame_count = 0
        self.print_interval = rospy.get_param("~print_interval", 10)  # Print every 10 frames

        rospy.loginfo("Zone Detector Node Started")
        rospy.loginfo("Subscribing to: /camera/image_raw")
        rospy.loginfo("Debug mode: %s", self.debug)
        rospy.loginfo("=" * 60)
        rospy.loginfo("PATH DETECTION STATUS:")
        rospy.loginfo("=" * 60)

    def image_cb(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)
            return

        # Convert BGR to HSV for better color segmentation
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # ---------- BLUE (SAFE PATH) DETECTION ----------
        blue_mask = cv2.inRange(hsv, self.low_blue, self.high_blue)
        
        # Apply morphological operations to reduce noise
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, self.kernel)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, self.kernel)

        # ---------- RED (INFECTION ZONE) DETECTION ----------
        # Red wraps around in HSV (0-10 and 170-180), so we need two masks
        red_mask1 = cv2.inRange(hsv, self.low_red1, self.high_red1)
        red_mask2 = cv2.inRange(hsv, self.low_red2, self.high_red2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        
        # Apply morphological operations to reduce noise
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, self.kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, self.kernel)

        # Count detected pixels
        blue_pixels = cv2.countNonZero(blue_mask)
        red_pixels = cv2.countNonZero(red_mask)

        # Determine detection status
        safe_detected = blue_pixels > self.safe_threshold
        infection_detected = red_pixels > self.infection_threshold

        # Print clear status to terminal
        self.frame_count += 1
        if self.frame_count % self.print_interval == 0:
            if infection_detected:
                status = "🔴 RED PATH (INFECTION) DETECTED"
                rospy.logwarn(status + " - %d pixels", red_pixels)
            elif safe_detected:
                status = "🔵 BLUE PATH (SAFE) DETECTED"
                rospy.loginfo(status + " - %d pixels", blue_pixels)
            else:
                status = "⚪ NONE - No path detected"
                rospy.loginfo(status + " (Blue: %d, Red: %d)", blue_pixels, red_pixels)

        # Publish detection status
        safe_bool = Bool()
        safe_bool.data = safe_detected
        self.safe_detected_pub.publish(safe_bool)

        infection_bool = Bool()
        infection_bool.data = infection_detected
        self.infection_detected_pub.publish(infection_bool)

        # Publish masks as Image messages (optional, for visualization in RViz)
        try:
            safe_mask_msg = self.bridge.cv2_to_imgmsg(blue_mask, "mono8")
            safe_mask_msg.header = msg.header
            self.safe_mask_pub.publish(safe_mask_msg)

            infection_mask_msg = self.bridge.cv2_to_imgmsg(red_mask, "mono8")
            infection_mask_msg.header = msg.header
            self.infection_mask_pub.publish(infection_mask_msg)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error publishing masks: %s", e)

        # Debug visualization
        if self.debug:
            # Create annotated frame for visualization
            vis_frame = frame.copy()
            
            # Overlay masks on original image (blue for safe, red for infection)
            vis_frame[blue_mask > 0] = [255, 0, 0]  # Blue overlay
            vis_frame[red_mask > 0] = [0, 0, 255]   # Red overlay
            
            # Blend with original for better visibility
            vis_frame = cv2.addWeighted(frame, 0.7, vis_frame, 0.3, 0)
            
            # Add text information
            cv2.putText(vis_frame, f"Blue (Safe): {blue_pixels} pixels", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(vis_frame, f"Red (Infection): {red_pixels} pixels", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Show windows
            cv2.imshow("Blue Safe Path", blue_mask)
            cv2.imshow("Red Infection Zone", red_mask)
            cv2.imshow("Original with Overlay", vis_frame)
            cv2.waitKey(1)



if __name__ == "__main__":
    try:
        detector = ZoneDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()

