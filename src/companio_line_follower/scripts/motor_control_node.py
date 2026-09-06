#!/usr/bin/env python3
"""
motor_control_node.py
======================
Converts /cmd_vel (Twist) into /lwheel_vtarget and /rwheel_vtarget (Float32)
for the Rhino RMCS-2303 motor driver via motor_driver_node.py (companio_base).

Physical motor convention (VERIFIED by user):
  positive velocity → turn_motor_ccw
  negative velocity → turn_motor_cw

  Right motor is mounted INVERTED, so its sign must be negated.

Physical direction truth:
  Forward : left=+CCW, right=-(+)=CW ✓
  Backward: left=-CW,  right=-(--)=CCW ✓
  Left rot: left=neg(CW), right=neg(CW) ✓  (angular.z>0)
  Right rot: left=pos(CCW), right=pos(CCW) ✓ (angular.z<0)

Formulas:
  left_speed  =  (v - w*L/2) * scale
  right_speed = -(v + w*L/2) * scale   ← negated for inverted mounting
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class MotorControlNode:

    def __init__(self):
        rospy.init_node('motor_control_node', anonymous=True)

        # Distance between wheels in meters (measure your robot)
        self.wheel_base  = rospy.get_param('~wheel_base',  0.2)
        # Scale: multiply m/s to get motor speed units sent to RMCS-2303
        # The motor_driver_ros_interface scales by velocity_scale_factor=19 internally,
        # so we pass rad/s-equivalent values here.
        self.speed_scale = rospy.get_param('~speed_scale', 2.0)
        self.max_speed   = rospy.get_param('~max_speed',  50.0)

        self.left_pub  = rospy.Publisher('/lwheel_vtarget', Float32, queue_size=1)
        self.right_pub = rospy.Publisher('/rwheel_vtarget', Float32, queue_size=1)
        self.cmd_sub   = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback, queue_size=1)

        rospy.loginfo(
            f"MotorControlNode ready | wheel_base={self.wheel_base}m "
            f"| scale={self.speed_scale} | max={self.max_speed}"
        )

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # Right motor is physically inverted (mounted backwards) → negate it.
        # angular.z > 0 = turn LEFT in ROS convention.
        left_speed  =  (v - w * self.wheel_base / 2.0) * self.speed_scale
        right_speed = -(v + w * self.wheel_base / 2.0) * self.speed_scale

        # Clamp to hardware limits
        left_speed  = max(-self.max_speed, min(self.max_speed,  left_speed))
        right_speed = max(-self.max_speed, min(self.max_speed, right_speed))

        self.left_pub.publish(Float32(data=left_speed))
        self.right_pub.publish(Float32(data=right_speed))


if __name__ == '__main__':
    try:
        node = MotorControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Publish zero on shutdown
        import rospy
        from geometry_msgs.msg import Twist
        try:
            pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
            pub.publish(Twist())
        except Exception:
            pass
