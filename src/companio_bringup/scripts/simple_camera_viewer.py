#!/usr/bin/env python3
"""
Simple Camera Viewer - displays RealSense camera feed.
Uses direct numpy conversion instead of cv_bridge to avoid
SystemError with cv_bridge_boost on mismatched OpenCV versions.
Press 'q' in the image window to quit.
"""
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
import sys

class SimpleCameraViewer:
    def __init__(self):
        rospy.init_node('simple_camera_viewer', anonymous=True)

        self.image_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.image_callback, queue_size=1)

        rospy.loginfo("Simple Camera Viewer Started!")
        rospy.loginfo("Waiting for images on /camera/color/image_raw ...")
        rospy.loginfo("Press 'q' in the image window to quit.")

    def image_callback(self, msg):
        try:
            # Convert ROS Image to numpy array directly (no cv_bridge needed)
            dtype = np.uint8
            channels = 3  # RGB8 / BGR8

            img = np.frombuffer(msg.data, dtype=dtype)
            img = img.reshape((msg.height, msg.width, channels))

            # RealSense publishes RGB; OpenCV expects BGR
            if msg.encoding == "rgb8":
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

            cv2.imshow("RealSense Camera Output", img)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                rospy.loginfo("Key 'q' pressed. Exiting...")
                rospy.signal_shutdown("User requested quit")
                cv2.destroyAllWindows()
                sys.exit(0)

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")


if __name__ == '__main__':
    try:
        viewer = SimpleCameraViewer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        cv2.destroyAllWindows()
