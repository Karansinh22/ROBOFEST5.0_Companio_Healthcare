#!/usr/bin/env python3
"""
line_follower_node.py
======================
Subscribes to /line/centroid (Float32) from hailo_line_detector_node.
  - NaN  = line not found → stop
  - -1.0 = line far LEFT  → turn left
  -  0.0 = line CENTRED   → straight
  - +1.0 = line far RIGHT → turn right

PID convention (matches existing working robot scripts):
  error  positive (line right) → angular.z negative → turn right ✓
  Achieved with angular_gain < 0  (same as angular_gain=-0.6 in line_following_black.py)
"""

import math
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class LineFollowerNode:

    def __init__(self):
        rospy.init_node('line_follower', anonymous=True)

        # PID gains – tuned for camera-based centroid (-1..+1 range)
        self.kp           = rospy.get_param('~kp',           1.2)
        self.ki           = rospy.get_param('~ki',           0.0)
        self.kd           = rospy.get_param('~kd',           0.15)
        self.base_speed   = rospy.get_param('~base_speed',   0.07)   # m/s
        # Negative: positive centroid (line right) → negative angular.z → turn right ✓
        self.angular_gain = rospy.get_param('~angular_gain', -1.5)
        self.max_angular  = rospy.get_param('~max_angular',   2.0)
        # Consecutive NaN frames before stopping
        self.lost_thresh  = rospy.get_param('~lost_thresh',   20)

        # PID state
        self.integral   = 0.0
        self.last_error = 0.0
        self.last_time  = rospy.Time.now()
        self._integral_max = 1.0

        # Line-lost counter — only increments on NaN, NOT on error=0.0
        self._lost_count = 0

        # ROS I/O
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.centroid_sub = rospy.Subscriber(
            '/line/centroid', Float32, self.centroid_callback, queue_size=1)

        # Safety watchdog: stop if detector goes silent
        self._last_msg_time = rospy.Time.now()
        rospy.Timer(rospy.Duration(0.2), self._watchdog)

        rospy.loginfo(
            f"LineFollower | kp={self.kp} kd={self.kd} "
            f"gain={self.angular_gain} speed={self.base_speed}m/s"
        )

    # ── Watchdog ──────────────────────────────────────────────────────────────

    def _watchdog(self, _):
        if (rospy.Time.now() - self._last_msg_time).to_sec() > 0.5:
            self._stop()

    # ── Main callback ─────────────────────────────────────────────────────────

    def centroid_callback(self, msg: Float32):
        self._last_msg_time = rospy.Time.now()
        error = msg.data

        # NaN means detector found NO line — increment lost counter
        if math.isnan(error):
            self._lost_count += 1
            if self._lost_count >= self.lost_thresh:
                rospy.logwarn_throttle(1, "Line LOST — stopping robot.")
                self.integral   = 0.0
                self.last_error = 0.0
                self._stop()
            return  # Do not reset integral — keep last state while briefly lost

        # Line found — reset lost counter
        self._lost_count = 0

        # Delta time
        now = rospy.Time.now()
        dt  = max((now - self.last_time).to_sec(), 0.005)
        self.last_time = now

        # PID computation
        self.integral = max(-self._integral_max,
                            min(self._integral_max, self.integral + error * dt))
        derivative = (error - self.last_error) / dt
        self.last_error = error

        pid_raw = (self.kp * error
                   + self.ki * self.integral
                   + self.kd * derivative)

        angular = max(-self.max_angular,
                      min(self.max_angular, pid_raw * self.angular_gain))

        self._publish(self.base_speed, angular)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _stop(self):
        self._publish(0.0, 0.0)

    def _publish(self, linear: float, angular: float):
        t = Twist()
        t.linear.x  = linear
        t.angular.z = angular
        self.cmd_pub.publish(t)


if __name__ == '__main__':
    try:
        node = LineFollowerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        try:
            pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
            pub.publish(Twist())
        except Exception:
            pass
