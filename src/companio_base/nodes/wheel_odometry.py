#!/usr/bin/env python3
"""
wheel_odometry.py
=================
Integrates the wheel encoder counts published by the motor driver into
nav_msgs/Odometry on /odom and broadcasts the odom -> base_footprint transform.

Subscribes
  lwheel_ticks_32bit / rwheel_ticks_32bit (std_msgs/UInt32)
Publishes
  odom (nav_msgs/Odometry) + TF odom -> base_footprint
"""

from math import cos, sin

import rospy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt32
from tf.broadcaster import TransformBroadcaster

from companio_base.kinematics import WheelOdometry


class WheelOdometryNode:
    def __init__(self):
        rospy.init_node("wheel_odometry")
        self.rate = float(rospy.get_param("~rate", 10.0))
        self.base_frame = rospy.get_param("~base_frame_id", "base_footprint")
        self.odom_frame = rospy.get_param("~odom_frame_id", "odom")
        self.publish_tf = bool(rospy.get_param("~publish_tf", True))

        self.odometry = WheelOdometry(
            ticks_per_meter=float(rospy.get_param("~ticks_meter", 607549)),
            base_width=float(rospy.get_param("~base_width", 0.22)),
            encoder_min=int(rospy.get_param("~encoder_min", 0)),
            encoder_max=int(rospy.get_param("~encoder_max", 4294967295)),
            invert_left=bool(rospy.get_param("~invert_left", True)),
            invert_right=bool(rospy.get_param("~invert_right", False)))

        rospy.Subscriber("lwheel_ticks_32bit", UInt32, self.left_callback)
        rospy.Subscriber("rwheel_ticks_32bit", UInt32, self.right_callback)
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.broadcaster = TransformBroadcaster()
        self._have_left = False
        self._have_right = False
        self._then = rospy.Time.now()
        rospy.loginfo("wheel_odometry ready (ticks/m=%.0f, base_width=%.3f m)",
                      self.odometry.ticks_per_meter, self.odometry.base_width)

    def left_callback(self, msg):
        self.odometry.update_left(msg.data)
        self._have_left = True

    def right_callback(self, msg):
        self.odometry.update_right(msg.data)
        self._have_right = True

    def update(self):
        now = rospy.Time.now()
        dt = (now - self._then).to_sec()
        self._then = now
        if not (self._have_left or self._have_right):
            rospy.loginfo_throttle(5.0, "wheel_odometry: waiting for encoder readings")
        x, y, theta = self.odometry.step(dt)

        q = Quaternion(0.0, 0.0, sin(theta / 2.0), cos(theta / 2.0))
        if self.publish_tf:
            self.broadcaster.sendTransform((x, y, 0.0), (q.x, q.y, q.z, q.w),
                                           now, self.base_frame, self.odom_frame)
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = self.odometry.v
        odom.twist.twist.angular.z = self.odometry.w
        self.odom_pub.publish(odom)

    def spin(self):
        loop = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            loop.sleep()


if __name__ == "__main__":
    try:
        WheelOdometryNode().spin()
    except rospy.ROSInterruptException:
        pass
