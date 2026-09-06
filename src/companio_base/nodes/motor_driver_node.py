#!/usr/bin/env python3
"""
motor_driver_node.py
====================
ROS interface for the two Rhino RMCS-2303 wheel motors.

Subscribes
  lwheel_vtarget / rwheel_vtarget  (std_msgs/Float32)  wheel speed targets
Publishes
  lwheel_ticks_32bit / rwheel_ticks_32bit (std_msgs/UInt32) raw encoder counts

Positive targets spin the motor counter-clockwise, negative targets clockwise.
Targets inside the dead-band brake the motor.  Both motors are braked when the
node shuts down.
"""

import sys
import time

import rospy
from std_msgs.msg import Float32, UInt32

from companio_base.rmcs2303 import Rmcs2303


class WheelMotor:
    """One wheel: RMCS-2303 controller + ROS topics."""

    def __init__(self, name, port, slave_id, ticks_topic, target_topic,
                 velocity_scale, deadband, settle_delay, gear_ratio):
        self.name = name
        self.velocity_scale = velocity_scale
        self.deadband = deadband
        self.settle_delay = settle_delay
        self._previous_scaled = None
        self.moving = False

        rospy.loginfo("%s motor: opening %s (slave %d)", name, port, slave_id)
        self.motor = Rmcs2303(port, slave_id, gear_ratio=gear_ratio)

        self.ticks_pub = rospy.Publisher(ticks_topic, UInt32, queue_size=1)
        rospy.Subscriber(target_topic, Float32, self.target_callback, queue_size=1)

    def target_callback(self, msg):
        target = float(msg.data)
        scaled = self.velocity_scale * target
        if scaled == self._previous_scaled:
            return

        if abs(target) < self.deadband:
            if self.moving:
                self.motor.brake()
                time.sleep(self.settle_delay)
                self.moving = False
                rospy.logdebug("%s: stopped", self.name)
        else:
            wanted = Rmcs2303.CCW if target > 0 else Rmcs2303.CW
            if self.motor.direction != wanted:
                # Load the speed target before enabling so the controller
                # never starts from a stale value.
                self.motor.set_speed(abs(scaled))
                if wanted == Rmcs2303.CCW:
                    self.motor.enable_ccw()
                else:
                    self.motor.enable_cw()
                time.sleep(self.settle_delay)
                rospy.logdebug("%s: direction -> %s", self.name,
                               "CCW" if wanted == Rmcs2303.CCW else "CW")
            self.motor.set_speed(abs(scaled))
            time.sleep(self.settle_delay)
            self.moving = True

        self._previous_scaled = scaled

    def publish_encoder(self):
        ticks = self.motor.read_encoder()
        if ticks is not None:
            self.ticks_pub.publish(UInt32(data=int(ticks)))

    def shutdown(self):
        try:
            self.motor.brake()
            time.sleep(self.settle_delay)
        except Exception as exc:  # pragma: no cover - hardware path
            rospy.logwarn("%s: brake on shutdown failed: %s", self.name, exc)


def main():
    rospy.init_node("motor_driver")

    left_port = rospy.get_param("~left_port", "/dev/motor_left")
    right_port = rospy.get_param("~right_port", "/dev/motor_right")
    left_id = int(rospy.get_param("~left_slave_id", 2))
    right_id = int(rospy.get_param("~right_slave_id", 5))
    velocity_scale = float(rospy.get_param("~velocity_scale", 19.0))
    deadband = float(rospy.get_param("~deadband", 0.02))
    settle_delay = float(rospy.get_param("~command_settle_delay", 0.02))
    gear_ratio = float(rospy.get_param("~gear_ratio", 100.0))
    encoder_rate = float(rospy.get_param("~encoder_rate", 20.0))

    try:
        left = WheelMotor("left", left_port, left_id, "lwheel_ticks_32bit",
                          "lwheel_vtarget", velocity_scale, deadband,
                          settle_delay, gear_ratio)
        right = WheelMotor("right", right_port, right_id, "rwheel_ticks_32bit",
                           "rwheel_vtarget", velocity_scale, deadband,
                           settle_delay, gear_ratio)
    except Exception as exc:
        rospy.logfatal("Cannot open motor controllers: %s", exc)
        rospy.logfatal("Check /dev/motor_left and /dev/motor_right (udev rules / container device links).")
        sys.exit(1)

    rospy.loginfo("Motor driver ready (velocity_scale=%.1f, encoder %.0f Hz)",
                  velocity_scale, encoder_rate)

    rate = rospy.Rate(encoder_rate)
    try:
        while not rospy.is_shutdown():
            left.publish_encoder()
            right.publish_encoder()
            rate.sleep()
    finally:
        rospy.loginfo("Braking motors before exit")
        left.shutdown()
        right.shutdown()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
