#!/usr/bin/env python3
"""
ROS Noetic node for line following using 5 IR sensors over UDP.
Listens for comma-separated sensor readings (0/1) sent to 127.0.0.1:5005,
computes a weighted angular correction, and publishes geometry_msgs/Twist
to /cmd_vel. The node stops the robot if data is missing,
invalid, or on shutdown.
"""
import socket
from typing import List, Optional
import rospy
from geometry_msgs.msg import Twist
class LineFollowerUDP:
    def __init__(self) -> None:
        # Parameters with safe defaults; override via ROS params if needed.
        self.listen_ip = rospy.get_param("~listen_ip", "127.0.0.1")
        self.listen_port = int(rospy.get_param("~listen_port", 5005))
        self.linear_speed = float(rospy.get_param("~linear_speed", 0.12))
        self.max_linear = float(rospy.get_param("~max_linear", 0.15))
        self.max_angular = float(rospy.get_param("~max_angular", 1.2))
        self.angular_gain = float(rospy.get_param("~angular_gain", 0.6))
        self.data_timeout = float(rospy.get_param("~data_timeout", 0.2))
        self.rate_hz = float(rospy.get_param("~rate_hz", 50.0))
        self.debug = bool(rospy.get_param("~debug", False))
        self.weights = [2, 1, 0, -1, -2]  # LEFT -> RIGHT
        self.expected_sensor_count = len(self.weights)
        self._pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.settimeout(0.0)  # Non-blocking
        self._bind_socket()
        self._last_valid_time = rospy.Time.now().to_sec()
        self._last_stop_sent = False
        rospy.loginfo(
            "line_follower_udp: listening on %s:%d, publishing to /cmd_vel",
            self.listen_ip,
            self.listen_port,
        )
    def _bind_socket(self) -> None:
        try:
            self._socket.bind((self.listen_ip, self.listen_port))
        except OSError as exc:
            rospy.logfatal("Failed to bind UDP socket on %s:%d: %s", self.listen_ip, self.listen_port, exc)
            raise
    def spin(self) -> None:
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            sensors = self._receive_sensors()
            now = rospy.Time.now().to_sec()
            if sensors is not None:
                self._last_valid_time = now
                self._last_stop_sent = False
                cmd = self._compute_twist(sensors)
                self._pub.publish(cmd)
            else:
                if now - self._last_valid_time > self.data_timeout:
                    if not self._last_stop_sent:
                        rospy.logwarn_throttle(1.0, "No valid UDP data; stopping robot.")
                        self._publish_stop()
                        self._last_stop_sent = True
            rate.sleep()
    def _receive_sensors(self) -> Optional[List[int]]:
        try:
            data, _ = self._socket.recvfrom(1024)
        except (socket.timeout, BlockingIOError):
            return None
        except OSError as exc:
            rospy.logwarn_throttle(5.0, "UDP receive error: %s", exc)
            return None
        try:
            text = data.decode("utf-8").strip()
        except UnicodeDecodeError:
            rospy.logwarn_throttle(1.0, "Received undecodable UDP packet.")
            return None
        parts = [p.strip() for p in text.split(",")]
        if len(parts) != self.expected_sensor_count:
            rospy.logwarn_throttle(1.0, "Expected %d sensors, got %d (%s).", self.expected_sensor_count, len(parts), text)
            return None
        readings: List[int] = []
        for p in parts:
            if p not in {"0", "1"}:
                rospy.logwarn_throttle(1.0, "Invalid sensor value '%s' in packet: %s", p, text)
                return None
            readings.append(int(p))
        if self.debug:
            rospy.logdebug("Sensors: %s", readings)
        return readings
    def _compute_twist(self, sensors: List[int]) -> Twist:
        active = sum(sensors)
        cmd = Twist()
        if active == 0:
            rospy.logwarn_throttle(1.0, "Line lost (all sensors 0); stopping.")
            return cmd  # zeroed Twist stops the robot
        weighted_sum = sum(w * s for w, s in zip(self.weights, sensors))
        error = weighted_sum / float(active)
        angular = self._clamp(error * self.angular_gain, -self.max_angular, self.max_angular)
        linear = self._clamp(self.linear_speed, 0.0, self.max_linear)
        cmd.linear.x = linear
        cmd.angular.z = angular
        if self.debug:
            rospy.logdebug("Sensor active: %d, error: %.3f, angular: %.3f", active, error, angular)
        else:
            rospy.loginfo_throttle(2.0, "Line following; ang: %.3f lin: %.3f", angular, linear)
        return cmd
    def _publish_stop(self) -> None:
        self._pub.publish(Twist())
    @staticmethod
    def _clamp(val: float, min_val: float, max_val: float) -> float:
        return max(min_val, min(max_val, val))
def main() -> None:
    rospy.init_node("line_follower_udp", anonymous=False)
    node = LineFollowerUDP()
    try:
        node.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        node._publish_stop()
        rospy.loginfo("line_follower_udp shut down cleanly.")
if __name__ == "__main__":
    main()
