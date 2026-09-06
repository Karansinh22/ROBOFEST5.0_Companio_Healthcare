#!/usr/bin/env python3
"""
ROS Noetic node for line following with fixed-path junction navigation.

Reads 5 digital IR sensors (0 = black/no reflection, 1 = white/reflection) from /dev/gpiochip0,
follows black line using weighted correction, detects junctions, and executes predefined
turn sequence to navigate to destination room.

Hardware: Raspberry Pi 5 (64-bit OS), 5 IR sensors via GPIO (libgpiod)
Motion control: geometry_msgs/Twist published to /cmd_vel

This version extends line_following_black.py with junction detection and path following.
"""

from typing import List, Optional, Tuple
import sys
import time
import signal

import rospy
from geometry_msgs.msg import Twist

try:
    import gpiod  # libgpiod-based GPIO (works on Ubuntu/RPi)
except ImportError as e:
    print(f"ERROR: gpiod not found. Install with: apt-get install python3-libgpiod")
    print(f"Error: {e}")
    sys.exit(1)


class LineFollowerPathNav:
    """Line follower with fixed-path junction navigation using finite state machine."""
    
    # Sensor weights for line following (LEFT -> RIGHT)
    WEIGHTS = [-2, -1, 0, 1, 2]
    
    # Finite State Machine states
    class State:
        FOLLOW_LINE = "FOLLOW_LINE"
        JUNCTION_DETECTED = "JUNCTION_DETECTED"
        CENTER_ON_JUNCTION = "CENTER_ON_JUNCTION"
        EXECUTE_TURN = "EXECUTE_TURN"
        SEARCH_LINE = "SEARCH_LINE"
        STOPPED = "STOPPED"
    
    def __init__(self) -> None:
        """Initialize the line follower with path navigation."""
        rospy.init_node("line_path_following", anonymous=False)
        
        # GPIO pin configuration
        self.pins = rospy.get_param("~pins", [17, 27, 22, 23, 24])
        self.expected_sensor_count = len(self.WEIGHTS)
        
        if len(self.pins) != self.expected_sensor_count:
            rospy.logfatal(
                "Expected %d GPIO pins, got %d (%s)",
                self.expected_sensor_count,
                len(self.pins),
                self.pins,
            )
            raise RuntimeError("Pin count does not match weights length")
        
        # Line following parameters (same as line_following_black.py)
        self.linear_speed = float(rospy.get_param("~linear_speed", 0.12))
        self.max_linear = float(rospy.get_param("~max_linear", 0.15))
        self.max_angular = float(rospy.get_param("~max_angular", 1.2))
        self.angular_gain = float(rospy.get_param("~angular_gain", -0.6))
        self.rate_hz = float(rospy.get_param("~rate_hz", 50.0))
        self.debug = bool(rospy.get_param("~debug", False))
        self.data_timeout = float(rospy.get_param("~data_timeout", 0.2))
        
        # Junction detection parameters
        self.junction_debounce_time = float(rospy.get_param("~junction_debounce_time", 0.5))  # seconds
        
        # Turn execution parameters (open-loop time-based)
        self.left_turn_time = float(rospy.get_param("~left_turn_time", 1.5))  # seconds
        self.right_turn_time = float(rospy.get_param("~right_turn_time", 1.5))  # seconds
        self.turn_speed = float(rospy.get_param("~turn_speed", 0.8))  # rad/s
        
        # Forward centering distance/time on junction
        self.center_forward_time = float(rospy.get_param("~center_forward_time", 0.3))  # seconds
        self.center_forward_speed = float(rospy.get_param("~center_forward_speed", 0.1))  # m/s
        
        # Search line parameters (after turn)
        self.search_forward_time = float(rospy.get_param("~search_forward_time", 0.5))  # seconds
        self.search_forward_speed = float(rospy.get_param("~search_forward_speed", 0.08))  # m/s
        
        # Fixed path sequence (configurable via ROS param)
        turn_sequence_param = rospy.get_param("~turn_sequence", "LEFT,RIGHT,STRAIGHT,LEFT")
        self.turn_sequence = [s.strip().upper() for s in turn_sequence_param.split(",")]
        
        # Validate turn sequence
        valid_turns = ["LEFT", "RIGHT", "STRAIGHT"]
        for turn in self.turn_sequence:
            if turn not in valid_turns:
                rospy.logfatal(f"Invalid turn in sequence: {turn}. Valid: {valid_turns}")
                raise ValueError(f"Invalid turn: {turn}")
        
        # State machine initialization
        self.state = self.State.FOLLOW_LINE
        self.junction_index = 0  # Current position in turn sequence
        self.last_junction_detection_time = 0.0
        self.junction_detection_start_time = 0.0
        self.center_start_time = 0.0
        self.turn_start_time = 0.0
        self.search_start_time = 0.0
        self.current_turn_action = None
        
        # Publisher
        self._pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self._shutdown_requested = False
        
        # Open gpiochip0 and claim inputs using gpiod (same as line_following_black.py)
        try:
            self._chip = gpiod.Chip("gpiochip0")
        except Exception as exc:
            rospy.logfatal("Cannot open gpiochip0 via gpiod: %s", exc)
            raise
        
        try:
            self._lines = self._chip.get_lines(self.pins)
            self._lines.request(
                consumer="line_path_following",
                type=gpiod.LINE_REQ_DIR_IN,
            )
        except Exception as exc:
            rospy.logfatal("Cannot request GPIO lines %s via gpiod: %s", self.pins, exc)
            try:
                self._chip.close()
            except Exception:
                pass
            raise
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("Line Path Follower initialized")
        rospy.loginfo(f"Turn sequence: {' -> '.join(self.turn_sequence)}")
        rospy.loginfo(f"Total junctions to execute: {len(self.turn_sequence)}")
        rospy.loginfo("=" * 60)
        
        # Start main control loop
        self.main_loop()
    
    def _read_sensors(self) -> Optional[List[int]]:
        """Read all IR sensors and return a list of 0/1 (0 = black/no reflection, 1 = white/reflection)."""
        readings: List[int] = []
        try:
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
            symbols = ["█" if v == 0 else "░" for v in readings]
            rospy.logdebug("Sensors: %s  (%s)", readings, " ".join(symbols))
        
        return readings
    
    def detect_junction(self, sensors: List[int]) -> Tuple[bool, str]:
        """
        Detect junction based on sensor width (number of black sensors).
        
        Returns:
            (is_junction, junction_type)
            junction_type: "L" for 3 sensors, "T" for 5 sensors, "" if not junction
        """
        black_count = sum(1 for s in sensors if s == 0)
        
        if black_count == 3:
            return (True, "L")
        elif black_count == 5:
            return (True, "T")
        else:
            return (False, "")
    
    def _compute_twist(self, sensors: List[int]) -> Twist:
        """
        Compute Twist from sensor readings (same logic as line_following_black.py).
        
        On this hardware, 0 = black (no light reflection), 1 = white (light reflection).
        This version MOVES when sensors detect BLACK (0).
        """
        active = sum(1 for s in sensors if s == 0)  # Count sensors seeing black
        cmd = Twist()
        
        if active == 0:
            rospy.logwarn_throttle(1.0, "Line lost (no sensors on black); stopping.")
            return cmd  # zero Twist
        
        # Invert sensors: treat 0 (black) as 1 (on line), 1 (white) as 0 (off line)
        line_bits = [1 - s for s in sensors]
        weighted_sum = sum(w * b for w, b in zip(self.WEIGHTS, line_bits))
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
        for _ in range(5):
            self._pub.publish(stop_cmd)
            rospy.sleep(0.01)
    
    @staticmethod
    def _clamp(val: float, min_val: float, max_val: float) -> float:
        return max(min_val, min(max_val, val))
    
    def main_loop(self) -> None:
        """Main control loop implementing finite state machine."""
        rate = rospy.Rate(self.rate_hz)
        self._last_valid_time = rospy.Time.now().to_sec()
        self._last_stop_sent = False
        
        try:
            while not rospy.is_shutdown() and not self._shutdown_requested:
                now = rospy.Time.now().to_sec()
                sensors = self._read_sensors()
                
                if sensors is None:
                    if not self._last_stop_sent:
                        rospy.logwarn_throttle(1.0, "Failed to read IR sensors; stopping.")
                        self._publish_stop()
                        self._last_stop_sent = True
                    rate.sleep()
                    continue
                
                # Check if line is active (at least one sensor on black)
                line_active = any(v == 0 for v in sensors)
                
                if line_active:
                    self._last_valid_time = now
                    self._last_stop_sent = False
                else:
                    # No line detected
                    if now - self._last_valid_time > self.data_timeout:
                        if not self._last_stop_sent:
                            rospy.logwarn("No valid sensor data; stopping.")
                            self._publish_stop()
                            self._last_stop_sent = True
                        rate.sleep()
                        continue
                
                # Finite State Machine
                if self.state == self.State.FOLLOW_LINE:
                    # Check end condition: all turns done AND line lost
                    if self.junction_index >= len(self.turn_sequence) and not line_active:
                        rospy.loginfo("=" * 60)
                        rospy.loginfo("All turns executed and line lost. Destination reached.")
                        rospy.loginfo("=" * 60)
                        self.state = self.State.STOPPED
                        self._publish_stop()
                        continue
                    
                    # Check for junction (only if we have turns remaining)
                    if self.junction_index < len(self.turn_sequence):
                        is_junction, junction_type = self.detect_junction(sensors)
                        
                        if is_junction:
                            # Debounce: junction must persist for debounce_time
                            if self.junction_detection_start_time == 0.0:
                                self.junction_detection_start_time = now
                            
                            if (now - self.junction_detection_start_time) >= self.junction_debounce_time:
                                # Junction confirmed
                                if (now - self.last_junction_detection_time) > self.junction_debounce_time:
                                    rospy.loginfo(
                                        f"[Junction {self.junction_index + 1}/{len(self.turn_sequence)}] "
                                        f"{junction_type}-junction detected. Action: {self.turn_sequence[self.junction_index]}"
                                    )
                                    self.state = self.State.JUNCTION_DETECTED
                                    self.last_junction_detection_time = now
                                    self.junction_detection_start_time = 0.0
                                else:
                                    if self.debug:
                                        rospy.logdebug("Junction debounce (too soon after last)")
                        else:
                            # Not a junction, reset detection timer
                            self.junction_detection_start_time = 0.0
                    
                    # Normal line following
                    if self.state == self.State.FOLLOW_LINE:
                        twist = self._compute_twist(sensors)
                        self._pub.publish(twist)
                
                elif self.state == self.State.JUNCTION_DETECTED:
                    # Pause line-following corrections and prepare to center
                    self._publish_stop()
                    rospy.sleep(0.1)
                    self.state = self.State.CENTER_ON_JUNCTION
                    self.center_start_time = now
                    rospy.loginfo("Centering robot on junction...")
                
                elif self.state == self.State.CENTER_ON_JUNCTION:
                    # Drive forward slightly to center robot on junction
                    elapsed = now - self.center_start_time
                    
                    if elapsed < self.center_forward_time:
                        twist = Twist()
                        twist.linear.x = self.center_forward_speed
                        twist.angular.z = 0.0
                        self._pub.publish(twist)
                    else:
                        # Centering complete, stop
                        self._publish_stop()
                        rospy.sleep(0.2)
                        self.state = self.State.EXECUTE_TURN
                        self.turn_start_time = now
                        self.current_turn_action = self.turn_sequence[self.junction_index]
                        rospy.loginfo(f"Executing turn: {self.current_turn_action}")
                
                elif self.state == self.State.EXECUTE_TURN:
                    # Execute turn based on current action (open-loop time-based)
                    elapsed = now - self.turn_start_time
                    
                    if self.current_turn_action == "LEFT":
                        if elapsed < self.left_turn_time:
                            twist = Twist()
                            twist.linear.x = 0.0
                            twist.angular.z = self.turn_speed  # Positive = left
                            self._pub.publish(twist)
                        else:
                            # Turn complete
                            self._publish_stop()
                            rospy.sleep(0.2)
                            self.state = self.State.SEARCH_LINE
                            self.search_start_time = now
                            rospy.loginfo("Turn complete. Searching for line...")
                    
                    elif self.current_turn_action == "RIGHT":
                        if elapsed < self.right_turn_time:
                            twist = Twist()
                            twist.linear.x = 0.0
                            twist.angular.z = -self.turn_speed  # Negative = right
                            self._pub.publish(twist)
                        else:
                            # Turn complete
                            self._publish_stop()
                            rospy.sleep(0.2)
                            self.state = self.State.SEARCH_LINE
                            self.search_start_time = now
                            rospy.loginfo("Turn complete. Searching for line...")
                    
                    elif self.current_turn_action == "STRAIGHT":
                        # For STRAIGHT, just move forward briefly
                        if elapsed < self.center_forward_time:
                            twist = Twist()
                            twist.linear.x = self.center_forward_speed
                            twist.angular.z = 0.0
                            self._pub.publish(twist)
                        else:
                            # Straight complete
                            self._publish_stop()
                            rospy.sleep(0.2)
                            self.junction_index += 1
                            self.state = self.State.FOLLOW_LINE
                            rospy.loginfo(f"Straight movement complete. Junction {self.junction_index}/{len(self.turn_sequence)}")
                
                elif self.state == self.State.SEARCH_LINE:
                    # Drive forward slowly to reacquire line
                    elapsed = now - self.search_start_time
                    
                    if elapsed < self.search_forward_time:
                        sensors_check = self._read_sensors()
                        if sensors_check and any(v == 0 for v in sensors_check):
                            # Line found!
                            self._publish_stop()
                            rospy.sleep(0.1)
                            self.junction_index += 1
                            self.state = self.State.FOLLOW_LINE
                            rospy.loginfo(f"Line reacquired. Junction {self.junction_index}/{len(self.turn_sequence)}")
                        else:
                            # Continue searching forward
                            twist = Twist()
                            twist.linear.x = self.search_forward_speed
                            twist.angular.z = 0.0
                            self._pub.publish(twist)
                    else:
                        # Search timeout - try to resume anyway
                        self._publish_stop()
                        rospy.sleep(0.1)
                        self.junction_index += 1
                        self.state = self.State.FOLLOW_LINE
                        rospy.logwarn("Line search timeout. Resuming line following...")
                
                elif self.state == self.State.STOPPED:
                    # Robot has stopped, publish zero commands continuously
                    self._publish_stop()
                    break
                
                rate.sleep()
        
        finally:
            # Cleanup on shutdown
            self._publish_stop()
            rospy.loginfo("Line path follower shutting down. Cleaning up GPIO...")
            try:
                try:
                    self._lines.release()
                except Exception:
                    pass
                self._chip.close()
            except Exception:
                pass
            rospy.loginfo("GPIO cleanup complete")


def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully."""
    rospy.signal_shutdown("Interrupted by user")


def main() -> None:
    """Main function."""
    # Register signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    node = None
    try:
        node = LineFollowerPathNav()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt received")
    except Exception as exc:
        rospy.logfatal("line_path_following crashed: %s", exc)
        time.sleep(0.5)
    finally:
        if node is not None:
            try:
                node._shutdown_requested = True
                node._publish_stop()
                time.sleep(0.1)
            except Exception:
                pass


if __name__ == "__main__":
    main()
