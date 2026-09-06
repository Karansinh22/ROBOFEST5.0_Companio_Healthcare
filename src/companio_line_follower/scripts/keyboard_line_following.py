#!/usr/bin/env python3
"""
Keyboard-Controlled Line Following with Location Detection

Features:
- Follows a single open-loop line
- Detects 4 location patches (where 4-5 IR sensors read BLACK)
- Keyboard commands: 1,2,3,4 (go to location), H (home), S (stop), F (forward), B (backward), C (continue)
- State machine with interrupt handling
- Noise reduction via debouncing
"""

from typing import List, Optional, Tuple
import sys
import time
import signal
import select
import termios
import tty

import rospy
from geometry_msgs.msg import Twist

try:
    import gpiod
except ImportError as e:
    print(f"ERROR: gpiod not found. Install with: apt-get install python3-libgpiod")
    print(f"Error: {e}")
    sys.exit(1)


class KeyboardLineFollower:
    """Line follower with location detection and keyboard control."""
    
    # Sensor weights for line following (LEFT -> RIGHT)
    WEIGHTS = [-2, -1, 0, 1, 2]
    
    # State machine states
    class State:
        IDLE = "IDLE"
        LINE_FOLLOW = "LINE_FOLLOW"
        ROTATING = "ROTATING"  # Rotating 180 degrees before backward navigation
        STOPPED = "STOPPED"
        MANUAL_OVERRIDE = "MANUAL_OVERRIDE"
    
    def __init__(self) -> None:
        """Initialize the keyboard-controlled line follower."""
        rospy.init_node("keyboard_line_following", anonymous=False)
        
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
        
        # Line following parameters
        self.linear_speed = float(rospy.get_param("~linear_speed", 0.12))
        self.max_linear = float(rospy.get_param("~max_linear", 0.15))
        self.max_angular = float(rospy.get_param("~max_angular", 1.2))
        self.angular_gain = float(rospy.get_param("~angular_gain", -0.6))
        self.rate_hz = float(rospy.get_param("~rate_hz", 50.0))
        self.debug = bool(rospy.get_param("~debug", False))
        self.data_timeout = float(rospy.get_param("~data_timeout", 0.2))
        
        # Location detection parameters (NOISE REDUCTION)
        self.location_debounce_time = float(rospy.get_param("~location_debounce_time", 0.3))  # seconds
        self.location_black_threshold = int(rospy.get_param("~location_black_threshold", 4))  # 4 or 5 sensors
        self.home_black_threshold = int(rospy.get_param("~home_black_threshold", 3))  # 3 sensors for home
        
        # State machine initialization
        self.state = self.State.IDLE
        self.current_location = 0  # Starts at home (0)
        self.target_location = None
        self.direction = "FORWARD"  # or "BACKWARD"
        self.location_count = 0  # Counter for detected locations
        
        # Location detection state (NOISE REDUCTION)
        self.location_detection_start_time = 0.0
        self.last_location_detection_time = 0.0
        self.already_detected = False
        self.last_location_patch_time = 0.0
        
        # Rotation state (for backward navigation)
        self.rotation_time = float(rospy.get_param("~rotation_time", 5))  # seconds
        self.rotation_speed = float(rospy.get_param("~rotation_speed", 1.0))  # rad/s
        self.rotation_start_time = 0.0
        self.ignore_sensors_during_rotation = True
        
        # Manual override state
        self.previous_state = None
        self.previous_target = None
        self.previous_direction = None
        
        # Publisher
        self._pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self._shutdown_requested = False
        
        # Keyboard input setup (only if stdin is a TTY)
        if sys.stdin.isatty():
            try:
                self.old_settings = termios.tcgetattr(sys.stdin)
                tty.setcbreak(sys.stdin.fileno())
                self.keyboard_enabled = True
            except (termios.error, OSError) as e:
                rospy.logwarn(f"Could not set terminal mode: {e}. Keyboard input may not work.")
                self.keyboard_enabled = False
                self.old_settings = None
        else:
            rospy.logwarn("stdin is not a TTY. Keyboard input disabled. Run with -it flag in docker exec.")
            self.keyboard_enabled = False
            self.old_settings = None
        
        # Open gpiochip0 and claim inputs using gpiod
        try:
            self._chip = gpiod.Chip("gpiochip0")
        except Exception as exc:
            rospy.logfatal("Cannot open gpiochip0 via gpiod: %s", exc)
            raise
        
        try:
            self._lines = self._chip.get_lines(self.pins)
            self._lines.request(
                consumer="keyboard_line_following",
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
        rospy.loginfo("Keyboard Line Follower initialized")
        rospy.loginfo("=" * 60)
        rospy.loginfo("Commands:")
        rospy.loginfo("  1, 2, 3, 4  → Go to location")
        rospy.loginfo("  H           → Go Home")
        rospy.loginfo("  S           → Stop")
        rospy.loginfo("  F           → Force forward")
        rospy.loginfo("  B           → Force backward")
        rospy.loginfo("  C           → Continue previous task")
        rospy.loginfo("  Q           → Quit")
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"Current location: {self.current_location} (Home)")
        rospy.loginfo("=" * 60)
        
        # Start main control loop
        self.main_loop()
    
    def _read_sensors(self) -> Optional[List[int]]:
        """Read all IR sensors and return a list of 0/1 (0 = black, 1 = white)."""
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
    
    def detect_location_patch(self, sensors: List[int]) -> Tuple[bool, bool]:
        """
        Detect location patch (4-5 sensors BLACK) or home (3 sensors BLACK).
        
        Returns:
            (is_location_patch, is_home)
        """
        black_count = sum(1 for s in sensors if s == 0)
        
        is_location = black_count >= self.location_black_threshold
        is_home = black_count >= self.home_black_threshold and black_count < self.location_black_threshold
        
        return (is_location, is_home)
    
    def _compute_twist(self, sensors: List[int], direction: str = "FORWARD") -> Twist:
        """
        Compute Twist from sensor readings.
        
        Args:
            sensors: List of sensor readings (0=black, 1=white)
            direction: "FORWARD" or "BACKWARD"
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
        
        # Reverse direction if going backward
        if direction == "BACKWARD":
            linear = -linear
            # Invert angular correction for backward movement
            angular = -angular
        
        cmd.linear.x = linear
        cmd.angular.z = angular
        
        if self.debug:
            rospy.logdebug(
                "Sensors: %s active: %d error: %.3f angular: %.3f linear: %.3f direction: %s",
                sensors,
                active,
                error,
                angular,
                linear,
                direction,
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
    
    def _get_keyboard_input(self) -> Optional[str]:
        """Get keyboard input without blocking."""
        if not self.keyboard_enabled:
            return None
        try:
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                return key.upper()
        except (OSError, ValueError):
            # stdin might not be available
            return None
        return None
    
    def _process_keyboard_command(self, key: str) -> None:
        """Process keyboard command and update state."""
        if key == 'Q':
            rospy.loginfo("Quit command received. Shutting down...")
            self._shutdown_requested = True
            return
        
        if key in ['1', '2', '3', '4']:
            target = int(key)
            if target == self.current_location:
                rospy.loginfo(f"Already at location {target}")
                return
            
            self.target_location = target
            self.location_count = self.current_location
            
            # Determine direction
            if target > self.current_location:
                self.direction = "FORWARD"
                rospy.loginfo(f"📍 Command: Go to Location {target}")
                rospy.loginfo(f"   Current: {self.current_location}, Target: {target}, Direction: FORWARD")
                self.state = self.State.LINE_FOLLOW
            elif target < self.current_location:
                self.direction = "BACKWARD"
                rospy.loginfo(f"📍 Command: Go to Location {target}")
                rospy.loginfo(f"   Current: {self.current_location}, Target: {target}, Direction: BACKWARD")
                rospy.loginfo("   Rotating 180 degrees first...")
                self.state = self.State.ROTATING
                self.rotation_start_time = rospy.Time.now().to_sec()
            else:
                self._publish_stop()
                self.state = self.State.STOPPED
                return
            
            self.already_detected = False
            self.location_detection_start_time = 0.0
        
        elif key == 'H':
            # Go home (location 0)
            if self.current_location == 0:
                rospy.loginfo("Already at home")
                return
            
            self.target_location = 0
            self.location_count = self.current_location
            self.direction = "BACKWARD"
            
            rospy.loginfo("🏠 Command: Go Home")
            rospy.loginfo(f"   Current: {self.current_location}, Target: 0, Direction: BACKWARD")
            rospy.loginfo("   Rotating 180 degrees first...")
            self.state = self.State.ROTATING
            self.rotation_start_time = rospy.Time.now().to_sec()
            self.already_detected = False
            self.location_detection_start_time = 0.0
        
        elif key == 'S':
            # Stop immediately
            rospy.loginfo("🛑 STOP command received")
            self._save_state()
            self._publish_stop()
            self.state = self.State.MANUAL_OVERRIDE
        
        elif key == 'F':
            # Force forward
            rospy.loginfo("⬆️  FORWARD command received")
            self._save_state()
            self.state = self.State.MANUAL_OVERRIDE
            twist = Twist()
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
            self._pub.publish(twist)
        
        elif key == 'B':
            # Force backward
            rospy.loginfo("⬇️  BACKWARD command received")
            self._save_state()
            self.state = self.State.MANUAL_OVERRIDE
            twist = Twist()
            twist.linear.x = -self.linear_speed
            twist.angular.z = 0.0
            self._pub.publish(twist)
        
        elif key == 'C':
            # Continue previous task
            if self.previous_state:
                rospy.loginfo("▶️  CONTINUE command received - Resuming previous task")
                self._restore_state()
            else:
                rospy.logwarn("No previous task to continue")
    
    def _save_state(self) -> None:
        """Save current state for continue command."""
        if self.state != self.State.MANUAL_OVERRIDE:
            self.previous_state = self.state
            self.previous_target = self.target_location
            self.previous_direction = self.direction
    
    def _restore_state(self) -> None:
        """Restore previous state."""
        if self.previous_state:
            self.state = self.previous_state
            self.target_location = self.previous_target
            self.direction = self.previous_direction
            self.already_detected = False
            self.location_detection_start_time = 0.0
            rospy.loginfo(f"   Restored: State={self.state}, Target={self.target_location}, Direction={self.direction}")
    
    def main_loop(self) -> None:
        """Main control loop implementing state machine."""
        rate = rospy.Rate(self.rate_hz)
        self._last_valid_time = rospy.Time.now().to_sec()
        self._last_stop_sent = False
        
        try:
            while not rospy.is_shutdown() and not self._shutdown_requested:
                now = rospy.Time.now().to_sec()
                
                # Always check for keyboard input (interrupt handling)
                key = self._get_keyboard_input()
                if key:
                    self._process_keyboard_command(key)
                
                # State Machine - Handle ROTATING state first (ignores IR sensors)
                if self.state == self.State.ROTATING:
                    # Rotate 180 degrees before backward navigation (ignore IR sensors for 5 seconds)
                    elapsed = now - self.rotation_start_time
                    
                    if elapsed < self.rotation_time:
                        # Rotate in place (left turn for 180 degrees)
                        # IR sensors are completely ignored during rotation period
                        # No sensor reading or line following logic is executed
                        twist = Twist()
                        twist.linear.x = 0.0
                        twist.angular.z = self.rotation_speed  # Positive = left turn
                        self._pub.publish(twist)
                        
                        if self.debug:
                            rospy.logdebug(f"Rotating... {elapsed:.2f}/{self.rotation_time:.2f}s (ignoring IR sensors)")
                    else:
                        # Rotation complete, stop and transition to line following
                        self._publish_stop()
                        rospy.sleep(0.2)
                        rospy.loginfo("✓ Rotation complete. Starting line following...")
                        self.state = self.State.LINE_FOLLOW
                        # Reset location detection
                        self.already_detected = False
                        self.location_detection_start_time = 0.0
                    
                    # Skip sensor reading during rotation
                    rate.sleep()
                    continue
                
                # Read sensors (only when not in ROTATING state)
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
                
                # State Machine (for non-ROTATING states)
                if self.state == self.State.IDLE:
                    # Wait for command - do nothing
                    self._publish_stop()
                
                elif self.state == self.State.LINE_FOLLOW:
                    # Follow line and detect locations
                    # Note: When direction is BACKWARD, we still follow line forward
                    # (after rotation, the robot is facing the opposite direction)
                    is_location, is_home = self.detect_location_patch(sensors)
                    
                    # Location detection with debouncing (NOISE REDUCTION)
                    if is_location or (is_home and self.target_location == 0):
                        # Start debounce timer
                        if self.location_detection_start_time == 0.0:
                            self.location_detection_start_time = now
                        
                        # Check if patch has been detected long enough
                        if (now - self.location_detection_start_time) >= self.location_debounce_time:
                            # Prevent multiple counts on same patch
                            if not self.already_detected:
                                self.already_detected = True
                                self.last_location_patch_time = now
                                
                                # Update location count based on direction
                                if self.direction == "FORWARD":
                                    self.location_count += 1
                                else:
                                    self.location_count -= 1
                                
                                # Check if home detected
                                if is_home and self.target_location == 0:
                                    self.current_location = 0
                                    rospy.loginfo("=" * 60)
                                    rospy.loginfo("🏠 HOME REACHED!")
                                    rospy.loginfo("=" * 60)
                                    self._publish_stop()
                                    self.state = self.State.STOPPED
                                    self.target_location = None
                                    continue
                                
                                # Check if target location reached
                                if self.location_count == self.target_location:
                                    self.current_location = self.target_location
                                    rospy.loginfo("=" * 60)
                                    rospy.loginfo(f"📍 LOCATION {self.target_location} REACHED!")
                                    rospy.loginfo("=" * 60)
                                    self._publish_stop()
                                    self.state = self.State.STOPPED
                                    self.target_location = None
                                    continue
                                else:
                                    rospy.loginfo(
                                        f"📍 Location patch detected! Count: {self.location_count}, Target: {self.target_location}"
                                    )
                    else:
                        # Reset detection if patch is lost
                        if self.already_detected:
                            # Only reset if we've moved away from patch
                            if now - self.last_location_patch_time > 0.5:
                                self.already_detected = False
                                self.location_detection_start_time = 0.0
                        else:
                            self.location_detection_start_time = 0.0
                    
                    # Normal line following
                    # After rotation, backward navigation uses forward line following
                    # (robot is facing opposite direction)
                    follow_direction = "FORWARD"  # Always follow forward after rotation
                    twist = self._compute_twist(sensors, follow_direction)
                    self._pub.publish(twist)
                
                elif self.state == self.State.STOPPED:
                    # Robot has stopped, wait for new command
                    self._publish_stop()
                
                elif self.state == self.State.MANUAL_OVERRIDE:
                    # Manual control mode - keep current velocity or stop
                    # User can give F, B, S, or C commands
                    # Velocity is set in _process_keyboard_command
                    pass
                
                rate.sleep()
        
        finally:
            # Cleanup on shutdown
            self._publish_stop()
            rospy.loginfo("Keyboard line follower shutting down. Cleaning up...")
            try:
                if self.keyboard_enabled and self.old_settings is not None:
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            except Exception:
                pass
            try:
                try:
                    self._lines.release()
                except Exception:
                    pass
                self._chip.close()
            except Exception:
                pass
            rospy.loginfo("Cleanup complete")


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
        node = KeyboardLineFollower()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt received")
    except Exception as exc:
        rospy.logfatal("keyboard_line_following crashed: %s", exc)
        import traceback
        traceback.print_exc()
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

