#!/usr/bin/env python3
"""
ROS Noetic node for controlling the left arm of a dual-arm hospital corridor robot.

Hardware: PCA9685 16-channel PWM servo driver
Left Arm Joints:
  - Shoulder[7]: 179 (down) to 0 (up), 90 (forward), Max Range: 155
  - Arm[8]: 90 (straight), 0 (inward), 180 (outward)
  - Elbow[9]: 90 (straight), 0 (down/inward), 180 (up), Range: 15 to 185
  - Elbow[9]: 90 (straight), 0 (down/inward), 180 (up), Range: 15 to 185
  - Gripper[12]: 175 (closed), 30 (open max)

Features:
  - Safety limits and collision avoidance
  - Smooth joint interpolation
  - ROS service interface
  - Joint state publishing

Uses smbus (like gpiod) for direct I2C communication - compatible with Ubuntu 20.04/Docker
"""

import time
import sys

try:
    import rospy
    from std_msgs.msg import Float64MultiArray
    from sensor_msgs.msg import JointState
except ImportError as e:
    print(f"ERROR: ROS packages not found. Make sure ROS Noetic is installed and sourced.")
    print(f"Error: {e}")
    sys.exit(1)

try:
    import smbus  # Standard I2C library (like gpiod for GPIO)
except ImportError as e:
    print(f"ERROR: smbus not found. Install with: apt-get install python3-smbus")
    print(f"Error: {e}")
    sys.exit(1)


class PCA9685:
    """PCA9685 PWM servo driver using direct I2C (smbus) - similar to gpiod approach."""
    
    # PCA9685 registers
    MODE1 = 0x00
    MODE2 = 0x01
    PRESCALE = 0xFE
    LED0_ON_L = 0x06
    LED0_ON_H = 0x07
    LED0_OFF_L = 0x08
    LED0_OFF_H = 0x09
    
    def __init__(self, bus=1, address=0x40):
        """Initialize PCA9685 on I2C bus."""
        self.bus = smbus.SMBus(bus)
        self.address = address
        self._initialize()
    
    def _initialize(self):
        """Initialize PCA9685 chip."""
        # Reset chip
        self.bus.write_byte_data(self.address, self.MODE1, 0x00)
        time.sleep(0.01)
        
        # Set prescaler for 50Hz (20ms period) - good for servos
        prescale = int(25000000.0 / (4096.0 * 50.0) - 1.0 + 0.5)
        old_mode = self.bus.read_byte_data(self.address, self.MODE1)
        new_mode = (old_mode & 0x7F) | 0x10  # Sleep mode
        self.bus.write_byte_data(self.address, self.MODE1, new_mode)
        self.bus.write_byte_data(self.address, self.PRESCALE, prescale)
        self.bus.write_byte_data(self.address, self.MODE1, old_mode)
        time.sleep(0.01)
        self.bus.write_byte_data(self.address, self.MODE1, old_mode | 0x80)  # Restart
    
    def set_pwm(self, channel, on, off):
        """Set PWM for a channel."""
        self.bus.write_byte_data(self.address, self.LED0_ON_L + 4 * channel, on & 0xFF)
        self.bus.write_byte_data(self.address, self.LED0_ON_H + 4 * channel, on >> 8)
        self.bus.write_byte_data(self.address, self.LED0_OFF_L + 4 * channel, off & 0xFF)
        self.bus.write_byte_data(self.address, self.LED0_OFF_H + 4 * channel, off >> 8)
    
    def set_angle(self, channel, angle, min_pulse=500, max_pulse=3000):
        """
        Set servo angle in degrees.
        
        Args:
            channel: Channel number (0-15)
            angle: Angle in degrees (0-180 typically)
            min_pulse: Minimum pulse width in microseconds (default 500)
            max_pulse: Maximum pulse width in microseconds (default 3000)
        """
        # Clamp angle to 0-180
        angle = max(0, min(180, angle))
        
        # Convert angle to pulse width
        pulse_width = min_pulse + (max_pulse - min_pulse) * (angle / 180.0)
        
        # Convert to 12-bit value (4096 steps for 20ms period at 50Hz)
        pulse = int(pulse_width * 4096.0 / 20000.0)
        
        # Set PWM (always start at 0, end at pulse value)
        self.set_pwm(channel, 0, pulse)


class LeftArmController:
    """Controller for the left arm with safety limits and collision avoidance."""
    
    # Joint channel mapping
    JOINT_CHANNELS = {
        'shoulder': 7,
        'arm': 8,
        'elbow': 9,
        'gripper': 12
    }
    
    # Joint names in order
    JOINT_NAMES = ['shoulder', 'arm', 'elbow', 'gripper']
    
    # Safety limits for each joint [min, max]
    JOINT_LIMITS = {
        'shoulder': [0, 155],      # 155 (down) to 0 (up) (increased from 130/150)
        'arm': [10, 150],          # 10 (inward) to 150 (outward)
        'elbow': [15, 185],        # 15 (down/inward) to 185 (up) (adjusted min and max)
        'gripper': [30, 175]       # 175 (closed) to 30 (open max) (increased from 160/170)
    }
    
    # Default safe positions (home position)
    HOME_POSITION = {
        'shoulder': 80,
        'arm': 85,
        'elbow': 95,
        'gripper': 160
    }
    
    def __init__(self):
        """Initialize the left arm controller."""
        rospy.init_node('left_arm_controller', anonymous=False)
        
        # Initialize PCA9685 using smbus (like gpiod approach)
        self.i2c_bus = rospy.get_param("~i2c_bus", 1)  # Usually 1 on Raspberry Pi
        self.i2c_address = rospy.get_param("~i2c_address", 0x40)
        
        try:
            self.pca9685 = PCA9685(bus=self.i2c_bus, address=self.i2c_address)
            rospy.loginfo(f"PCA9685 initialized on I2C bus {self.i2c_bus}, address 0x{self.i2c_address:02X}")
        except Exception as exc:
            rospy.logfatal(f"Cannot initialize PCA9685: {exc}")
            rospy.logfatal("Make sure I2C is enabled and PCA9685 is connected")
            raise
        
        # Current joint positions
        self.current_positions = self.HOME_POSITION.copy()
        
        # Movement parameters
        self.move_speed = rospy.get_param("~move_speed", 0.5)  # seconds per degree
        self.interpolation_steps = rospy.get_param("~interpolation_steps", 10)
        
        # Publishers
        self.joint_state_pub = rospy.Publisher(
            '/left_arm/joint_states',
            JointState,
            queue_size=10
        )
        
        # Subscribers
        rospy.Subscriber(
            '/left_arm/joint_commands',
            Float64MultiArray,
            self.joint_command_callback,
            queue_size=1
        )
        
        # Move to home position on startup
        rospy.loginfo("Moving left arm to home position...")
        self.move_to_position(self.HOME_POSITION, smooth=True)
        rospy.loginfo("Left arm controller initialized and ready")
        
        # Start joint state publisher
        self.publish_joint_states()
    
    def clamp_angle(self, joint_name, angle):
        """Clamp angle to joint limits."""
        limits = self.JOINT_LIMITS[joint_name]
        return max(limits[0], min(limits[1], angle))
    
    def check_collision_avoidance(self, target_positions):
        """
        Check for potential collisions and adjust if necessary.
        
        Basic collision avoidance rules:
        1. Prevent elbow from going too low when shoulder is up
        2. Prevent arm from extending too far when elbow is bent
        3. Prevent wrist from extreme angles when gripper is open
        """
        safe_positions = target_positions.copy()
        
        # Rule 1: If shoulder is high (> 120), limit elbow to prevent hitting body
        if safe_positions['shoulder'] > 120:
            if safe_positions['elbow'] < 60:
                safe_positions['elbow'] = 60
                rospy.logwarn("Collision avoidance: Limited elbow angle (shoulder too high)")
        
        # Rule 2: If elbow is very bent (< 30), limit arm extension
        if safe_positions['elbow'] < 30:
            if safe_positions['arm'] < 45 or safe_positions['arm'] > 135:
                safe_positions['arm'] = 90
                rospy.logwarn("Collision avoidance: Limited arm angle (elbow too bent)")
        
        # Rule 3 removed because wrist joint no longer exists
        
        # Rule 4: Prevent shoulder from going too low when arm is extended outward
        if safe_positions['arm'] > 150:
            if safe_positions['shoulder'] < 30:
                safe_positions['shoulder'] = 30
                rospy.logwarn("Collision avoidance: Limited shoulder angle (arm extended)")
        
        return safe_positions
    
    def move_joint(self, joint_name, target_angle, smooth=True):
        """
        Move a single joint to target angle.
        
        Args:
            joint_name: Name of the joint
            target_angle: Target angle in degrees
            smooth: If True, interpolate smoothly
        """
        if joint_name not in self.JOINT_CHANNELS:
            rospy.logerr(f"Unknown joint: {joint_name}")
            return False
        
        # Clamp to limits
        target_angle = self.clamp_angle(joint_name, target_angle)
        channel = self.JOINT_CHANNELS[joint_name]
        current_angle = self.current_positions[joint_name]
        
        if smooth and abs(target_angle - current_angle) > 1:
            step_size = 5.0
            delay = 0.25
            
            while abs(target_angle - current_angle) > 0.1:
                if target_angle > current_angle:
                    current_angle = min(target_angle, current_angle + step_size)
                else:
                    current_angle = max(target_angle, current_angle - step_size)
                    
                self.pca9685.set_angle(channel, current_angle)
                self.current_positions[joint_name] = current_angle
                time.sleep(delay)
        else:
            self.pca9685.set_angle(channel, target_angle)
            self.current_positions[joint_name] = target_angle
            if smooth:
                time.sleep(0.25)
        return True
    
    def move_to_position(self, target_positions, smooth=True):
        """
        Move all joints to target positions with collision avoidance.
        
        Args:
            target_positions: Dict of joint_name -> angle
            smooth: If True, interpolate smoothly
        """
        # Apply safety limits
        safe_targets = {}
        for joint_name, angle in target_positions.items():
            if joint_name in self.JOINT_CHANNELS:
                safe_targets[joint_name] = self.clamp_angle(joint_name, angle)
            else:
                rospy.logwarn(f"Ignoring unknown joint: {joint_name}")
        
        # Apply collision avoidance
        safe_targets = self.check_collision_avoidance(safe_targets)
        
        # Enforce safe sequential movement order: Arm -> Shoulder -> Elbow
        motion_sequence = ['arm', 'shoulder', 'elbow', 'gripper']
        for joint_name in motion_sequence:
            if joint_name in safe_targets:
                self.move_joint(joint_name, safe_targets[joint_name], smooth=smooth)
        
        # Small delay after movement
        time.sleep(0.1)
    
    def joint_command_callback(self, msg):
        """
        Handle joint command messages.
        
        Expected format: Float64MultiArray with 4 values
        [shoulder, arm, elbow, gripper]
        """
        if len(msg.data) != 4:
            rospy.logerr(f"Invalid joint command: expected 4 values, got {len(msg.data)}")
            return
        
        target_positions = {
            'shoulder': float(msg.data[0]),
            'arm': float(msg.data[1]),
            'elbow': float(msg.data[2]),
            'gripper': float(msg.data[3])
        }
        
        rospy.loginfo(f"Received joint command: {target_positions}")
        self.move_to_position(target_positions, smooth=True)
    
    def publish_joint_states(self):
        """Publish current joint states."""
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            joint_state = JointState()
            joint_state.header.stamp = rospy.Time.now()
            joint_state.name = [f"left_{name}" for name in self.JOINT_NAMES]
            joint_state.position = [self.current_positions[name] for name in self.JOINT_NAMES]
            joint_state.velocity = [0.0] * len(self.JOINT_NAMES)
            joint_state.effort = [0.0] * len(self.JOINT_NAMES)
            
            self.joint_state_pub.publish(joint_state)
            rate.sleep()
    
    def move_to_home(self):
        """Move arm to safe home position."""
        rospy.loginfo("Moving to home position...")
        self.move_to_position(self.HOME_POSITION, smooth=True)
    
    def shutdown(self):
        """Clean shutdown - move to home position."""
        rospy.loginfo("Shutting down left arm controller...")
        self.move_to_home()
        rospy.loginfo("Left arm controller shut down")


def main():
    """Main function."""
    controller = None
    try:
        controller = LeftArmController()
        
        # Register shutdown handler
        rospy.on_shutdown(controller.shutdown)
        
        # Keep node alive
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt received")
    except Exception as exc:
        rospy.logfatal(f"Left arm controller crashed: {exc}")
    finally:
        if controller is not None:
            controller.shutdown()


if __name__ == "__main__":
    main()
