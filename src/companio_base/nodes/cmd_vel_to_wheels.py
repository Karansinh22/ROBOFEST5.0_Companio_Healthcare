#!/usr/bin/env python3
"""
cmd_vel_to_wheels.py
====================
Converts geometry_msgs/Twist on /cmd_vel into left/right wheel speed targets
(std_msgs/Float32 on lwheel_vtarget / rwheel_vtarget) for the motor driver.

Targets are re-published at ~rate Hz for ~timeout_ticks cycles after the last
Twist, then the node goes idle.  Set ~publish_zero_on_timeout to true to send
an explicit stop when commands stop arriving.
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

from companio_base.kinematics import DifferentialKinematics


class CmdVelToWheels:
    def __init__(self):
        rospy.init_node("cmd_vel_to_wheels")
        base_width = float(rospy.get_param("~base_width", 0.22))
        self.rate = float(rospy.get_param("~rate", 5.0))
        self.timeout_ticks = int(rospy.get_param("~timeout_ticks", 2))
        self.zero_on_timeout = bool(rospy.get_param("~publish_zero_on_timeout", False))
        self.kinematics = DifferentialKinematics(
            base_width,
            invert_right=bool(rospy.get_param("~invert_right", True)),
            invert_left=bool(rospy.get_param("~invert_left", False)))

        self.pub_left = rospy.Publisher("lwheel_vtarget", Float32, queue_size=1)
        self.pub_right = rospy.Publisher("rwheel_vtarget", Float32, queue_size=1)
        rospy.Subscriber("/cmd_vel", Twist, self.twist_callback, queue_size=1)

        self.linear = 0.0
        self.angular = 0.0
        self.ticks_since_target = self.timeout_ticks
        rospy.loginfo("cmd_vel_to_wheels ready (base_width=%.3f m, %.0f Hz)",
                      base_width, self.rate)

    def twist_callback(self, msg):
        self.linear = msg.linear.x
        self.angular = msg.angular.z
        self.ticks_since_target = 0

    def publish(self, linear, angular):
        left, right = self.kinematics.wheel_targets(linear, angular)
        self.pub_left.publish(Float32(data=left))
        self.pub_right.publish(Float32(data=right))

    def spin(self):
        loop = rospy.Rate(self.rate)
        idle = rospy.Rate(5)
        while not rospy.is_shutdown():
            while not rospy.is_shutdown() and self.ticks_since_target < self.timeout_ticks:
                self.publish(self.linear, self.angular)
                self.ticks_since_target += 1
                loop.sleep()
            if self.zero_on_timeout and self.ticks_since_target == self.timeout_ticks:
                self.publish(0.0, 0.0)
                self.ticks_since_target += 1
            idle.sleep()


if __name__ == "__main__":
    try:
        CmdVelToWheels().spin()
    except rospy.ROSInterruptException:
        pass
