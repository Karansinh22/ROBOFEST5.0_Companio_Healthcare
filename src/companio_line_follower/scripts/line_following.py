#!/usr/bin/env python3
"""
ROS Noetic node for line following using 5 IR sensors via GPIO (gpiod).

Reads 5 digital IR sensors (0 = black, 1 = white) from /dev/gpiochip0,
computes a weighted angular correction, and publishes geometry_msgs/Twist
to /cmd_vel. The node stops the robot if the line is lost or on shutdown.
"""

from typing import List, Optional
import sys
import time
import signal

import rospy
from geometry_msgs.msg import Twist

import gpiod  # libgpiod-based GPIO (works on Ubuntu/RPi)


class LineFollowerGPIO:
    def __init__(self) -> None:
        # Parameters with safe defaults; override via ROS params if needed.
        # GPIO pins: BCM numbers, ordered LEFT -> RIGHT
        self.pins = rospy.get_param("~pins", [17, 27, 22, 23, 24])

        self.linear_speed = float(rospy.get_param("~linear_speed", 0.12))
        self.max_linear = float(rospy.get_param("~max_linear", 0.15))
        self.max_angular = float(rospy.get_param("~max_angular", 1.2))
        # negative to steer toward the side where the line is detected
        self.angular_gain = float(rospy.get_param("~angular_gain", -0.6))
        self.rate_hz = float(rospy.get_param("~rate_hz", 50.0))
        self.debug = bool(rospy.get_param("~debug", False))

        # Weights: LEFT -> RIGHT. With negative gain above, this makes the robot
        # turn toward the side where the line is detected.
        self.weights = [-2, -1, 0, 1, 2]
        self.expected_sensor_count = len(self.weights)

        if len(self.pins) != self.expected_sensor_count:
            rospy.logfatal(
                "Expected %d GPIO pins, got %d (%s)",
                self.expected_sensor_count,
                len(self.pins),
                self.pins,
            )
            raise RuntimeError("Pin count does not match weights length")

        self._pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self._shutdown_requested = False

        # Open gpiochip0 and claim inputs using gpiod
        try:
            # "gpiochip0" is the typical name, equivalent to index 0
            self._chip = gpiod.Chip("gpiochip0")
        except Exception as exc:
            rospy.logfatal("Cannot open gpiochip0 via gpiod: %s", exc)
            raise

        try:
            # Request all pins as a single line set, configured as inputs
            self._lines = self._chip.get_lines(self.pins)
            self._lines.request(
                consumer="line_follower_gpio",
                type=gpiod.LINE_REQ_DIR_IN,
            )
        except Exception as exc:
            rospy.logfatal("Cannot request GPIO lines %s via gpiod: %s", self.pins, exc)
            try:
                self._chip.close()
            except Exception:
                pass
            raise


    def spin(self) -> None:
        rate = rospy.Rate(self.rate_hz)
        try:
            while not rospy.is_shutdown() and not self._shutdown_requested:
                sensors = self._read_sensors()
                if sensors is None:
                    # If GPIO read fails for some reason, stop the robot.
                    rospy.logwarn_throttle(1.0, "Failed to read IR sensors; stopping.")
                    self._publish_stop()
                else:
                    cmd = self._compute_twist(sensors)
                    self._pub.publish(cmd)

                rate.sleep()
        finally:
            # Always stop the robot and release GPIO on shutdown
            self._publish_stop()
            # Small delay to ensure stop message is sent
            time.sleep(0.1)
            try:
                # Release lines and close chip
                try:
                    self._lines.release()
                except Exception:
                    pass
                self._chip.close()
            except Exception:
                pass

    def _read_sensors(self) -> Optional[List[int]]:
        """Read all IR sensors and return a list of 0/1 (0 = black, 1 = white)."""
        readings: List[int] = []
        try:
            # Read all values at once; returns a list aligned with self.pins
            vals = self._lines.get_values()
            for val in vals:
                if val not in (0, 1):
                    rospy.logwarn_throttle(
                        1.0,
                        "Unexpected GPIO value %s",
                        val,
                    )
                    return None
                readings.append(int(val))
        except Exception as exc:
            rospy.logwarn_throttle(1.0, "GPIO read error via gpiod: %s", exc)
            return None

        if self.debug:
            # Visual representation: █ = black (0), ░ = white (1)
            symbols = ["█" if v == 0 else "░" for v in readings]
            rospy.logdebug("Sensors: %s  (%s)", readings, " ".join(symbols))

        return readings

    def _compute_twist(self, sensors: List[int]) -> Twist:
        """
        Compute Twist from sensor readings.

        On this hardware, 1 = black (line), 0 = white (background).
        """
        # Count sensors seeing the line (value 1)
        active = sum(1 for s in sensors if s == 1)
        cmd = Twist()

        if active == 0:
            # No sensor sees the line → stop
            rospy.logwarn_throttle(1.0, "Line lost (no sensors on black); stopping.")
            return cmd  # zero Twist

        # Treat sensors directly as line bits: 1 = on line, 0 = off
        line_bits = sensors[:]
        weighted_sum = sum(w * b for w, b in zip(self.weights, line_bits))
        error = weighted_sum / float(active)

        angular = self._clamp(error * self.angular_gain, -self.max_angular, self.max_angular)
        linear = self._clamp(self.linear_speed, 0.0, self.max_linear)

        cmd.linear.x = linear
        cmd.angular.z = angular

        if self.debug:
            rospy.logdebug(
                "Sensors: %s line_bits: %s active: %d error: %.3f angular: %.3f linear: %.3f",
                sensors,
                line_bits,
                active,
                error,
                angular,
                linear,
            )

        return cmd

    def _publish_stop(self) -> None:
        """Publish stop command multiple times to ensure it's received."""
        stop_cmd = Twist()
        for _ in range(5):  # Publish 5 times to ensure it gets through
            self._pub.publish(stop_cmd)
            rospy.sleep(0.01)  # Small delay between publishes

    @staticmethod
    def _clamp(val: float, min_val: float, max_val: float) -> float:
        return max(min_val, min(max_val, val))


def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully."""
    rospy.signal_shutdown("Interrupted by user")


def main() -> None:
    rospy.init_node("line_follower_gpio", anonymous=False)
    rospy.loginfo("Starting line_follower_gpio node...")
    
    # Register signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    node = None
    try:
        node = LineFollowerGPIO()
        rospy.loginfo("Line follower initialized, starting control loop...")
        node.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        rospy.logfatal("line_follower_gpio crashed: %s", exc)
        # brief delay to ensure log flush
        time.sleep(0.5)
    finally:
        if node is not None:
            try:
                node._shutdown_requested = True
                node._publish_stop()
                time.sleep(0.1)  # Ensure stop message is sent
            except Exception:
                pass


if __name__ == "__main__":
    main()