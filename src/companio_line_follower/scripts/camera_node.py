#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

class CameraNode:
    def __init__(self):
        rospy.init_node('camera_node', anonymous=True)
        
        # Parameters
        self.device_id = rospy.get_param('~device_id', 0)
        self.width = rospy.get_param('~width', 640)
        self.height = rospy.get_param('~height', 480)
        self.fps = rospy.get_param('~fps', 30)
        
        # Publishers
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Camera initialization
        self.cap = None
        self.connect_camera()

    def connect_camera(self):
        while not rospy.is_shutdown():
            rospy.loginfo(f"Attempting to connect to camera {self.device_id}...")
            self.cap = cv2.VideoCapture(self.device_id)
            if self.cap.is_opened():
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.cap.set(cv2.CAP_PROP_FPS, self.fps)
                rospy.loginfo("Camera connected successfully.")
                break
            else:
                rospy.logwarn("Failed to open camera. Retrying in 2 seconds...")
                time.sleep(2)

    def run(self):
        rate = rospy.Rate(self.fps)
        while not rospy.is_shutdown():
            if self.cap is None or not self.cap.is_opened():
                self.connect_camera()

            ret, frame = self.cap.read()
            if ret:
                try:
                    # Convert to ROS message
                    msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = "camera_link"
                    self.image_pub.publish(msg)
                except Exception as e:
                    rospy.logerr(f"Error publishing image: {e}")
            else:
                rospy.logwarn("Lost camera frame. Reconnecting...")
                self.cap.release()
                self.connect_camera()
            
            rate.sleep()

        if self.cap:
            self.cap.release()

if __name__ == '__main__':
    try:
        node = CameraNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
