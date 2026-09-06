#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class LineFollowingNode:
    def __init__(self):
        rospy.init_node('line_following_node', anonymous=True)
        
        # Parameters
        self.kp = rospy.get_param('~kp', 0.005)
        self.ki = rospy.get_param('~ki', 0.0)
        self.kd = rospy.get_param('~kd', 0.001)
        self.base_speed = rospy.get_param('~base_speed', 0.2)
        
        # PID state
        self.last_error = 0
        self.integral = 0
        
        # Publishers/Subscribers
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.debug_pub = rospy.Publisher('/ai/line_debug', Image, queue_size=1)
        
        self.bridge = CvBridge()
        rospy.loginfo("Line following node initialized.")

    def image_callback(self, msg):
        try:
            # 1. Convert Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            height, width = cv_image.shape[:2]
            
            # 2. Preprocessing
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)
            
            # 3. Canny Edge Detection
            edges = cv2.Canny(blur, 50, 150)
            
            # 4. ROI (Region of Interest) - Bottom half
            mask = np.zeros_like(edges)
            roi_vertices = np.array([[(0, height), (width, height), (width, height//2), (0, height//2)]], dtype=np.int32)
            cv2.fillPoly(mask, roi_vertices, 255)
            masked_edges = cv2.bitwise_and(edges, mask)
            
            # 5. Detect Line Centroid using Moments
            M = cv2.moments(masked_edges)
            
            if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                
                # Draw for debug
                cv2.circle(cv_image, (cx, cy), 10, (0, 255, 0), -1)
                cv2.line(cv_image, (width//2, 0), (width//2, height), (255, 0, 0), 2)
                
                # 6. PID Control Logic
                error = cx - width//2
                
                # PID calculation
                P = self.kp * error
                self.integral += error
                I = self.ki * self.integral
                D = self.kd * (error - self.last_error)
                
                steering = P + I + D
                self.last_error = error
                
                # 7. Publish Motor Commands
                self.publish_twist(self.base_speed, -steering)
            else:
                # Fallback: Stop if line is lost
                rospy.logwarn_throttle(1, "Line not detected! Stopping.")
                self.publish_twist(0.0, 0.0)
            
            # Publish debug image
            debug_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.debug_pub.publish(debug_msg)

        except Exception as e:
            rospy.logerr(f"Line following error: {e}")

    def publish_twist(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    try:
        node = LineFollowingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
