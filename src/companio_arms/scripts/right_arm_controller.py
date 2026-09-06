#!/usr/bin/env python3
"""
ROS Noetic node for controlling the right arm of a dual-arm hospital corridor robot.

Hardware: PCA9685 16-channel PWM servo driver
Right Arm Joints:
  - Shoulder[2]: 179 (down) to 0 (up), 90 (forward)
  - Arm[3]: 90 (straight), 0 (inward), 180 (outward)
  - Elbow[4]: 90 (straight), 0 (down/inward), 180 (up)
  - Elbow[4]: 90 (straight), 0 (down/inward), 180 (up)
  - Gripper[6]: 175 (closed), 30 (open max)

Features:
  - Safety limits and collision avoidance
  - Smooth joint interpolation
  - ROS service interface
  - Joint state publishing

Uses smbus for direct I2C communication (like gpiod for GPIO), compatible with Ubuntu 20.04/Docker.
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
    import smbus
except ImportError as e:
    print(f"ERROR: smbus not found. Install with: apt-get install python3-smbus")
    print(f"Error: {e}")
    sys.exit(1)


class PCA9685:
    MODE1 = 0x00
    MODE2 = 0x01
    PRESCALE = 0xFE
    LED0_ON_L = 0x06
    LED0_ON_H = 0x07
    LED0_OFF_L = 0x08
    LED0_OFF_H = 0x09
    def __init__(self, bus=1, address=0x40):
        self.bus = smbus.SMBus(bus)
        self.address = address
        self._initialize()
    def _initialize(self):
        self.bus.write_byte_data(self.address, self.MODE1, 0x00)
        time.sleep(0.01)
        prescale = int(25000000.0 / (4096.0 * 50.0) - 1.0 + 0.5)
        old_mode = self.bus.read_byte_data(self.address, self.MODE1)
        new_mode = (old_mode & 0x7F) | 0x10
        self.bus.write_byte_data(self.address, self.MODE1, new_mode)
        self.bus.write_byte_data(self.address, self.PRESCALE, prescale)
        self.bus.write_byte_data(self.address, self.MODE1, old_mode)
        time.sleep(0.01)
        self.bus.write_byte_data(self.address, self.MODE1, old_mode | 0x80)
    def set_pwm(self, channel, on, off):
        self.bus.write_byte_data(self.address, self.LED0_ON_L + 4 * channel, on & 0xFF)
        self.bus.write_byte_data(self.address, self.LED0_ON_H + 4 * channel, on >> 8)
        self.bus.write_byte_data(self.address, self.LED0_OFF_L + 4 * channel, off & 0xFF)
        self.bus.write_byte_data(self.address, self.LED0_OFF_H + 4 * channel, off >> 8)
    def set_angle(self, channel, angle, min_pulse=500, max_pulse=3000):
        angle = max(0, min(180, angle))
        pulse_width = min_pulse + (max_pulse - min_pulse) * (angle / 180.0)
        pulse = int(pulse_width * 4096.0 / 20000.0)
        self.set_pwm(channel, 0, pulse)


class RightArmController:
    JOINT_CHANNELS = {
        'shoulder': 0,
        'arm': 1,
        'elbow': 3,
        'gripper': 5
    }
    JOINT_NAMES = ['shoulder', 'arm', 'elbow', 'gripper']
    JOINT_LIMITS = {
        'shoulder': [0, 150],
        'arm': [0, 150],
        'elbow': [5, 160],
        'gripper': [30, 175]
    }
    HOME_POSITION = {
        'shoulder': 90,
        'arm': 90,
        'elbow': 90,
        'gripper': 160
    }
    def __init__(self):
        rospy.init_node('right_arm_controller', anonymous=False)
        self.i2c_bus = rospy.get_param("~i2c_bus", 1)
        self.i2c_address = rospy.get_param("~i2c_address", 0x40)
        try:
            self.pca9685 = PCA9685(bus=self.i2c_bus, address=self.i2c_address)
            rospy.loginfo(f"PCA9685 initialized on I2C bus {self.i2c_bus}, address 0x{self.i2c_address:02X}")
        except Exception as exc:
            rospy.logfatal(f"Cannot initialize PCA9685: {exc}")
            rospy.logfatal("Make sure I2C is enabled and PCA9685 is connected")
            raise
        self.current_positions = self.HOME_POSITION.copy()
        self.move_speed = rospy.get_param("~move_speed", 0.5)
        self.interpolation_steps = rospy.get_param("~interpolation_steps", 10)
        self.joint_state_pub = rospy.Publisher(
            '/right_arm/joint_states',
            JointState,
            queue_size=10)
        rospy.Subscriber(
            '/right_arm/joint_commands',
            Float64MultiArray,
            self.joint_command_callback,
            queue_size=1)
        rospy.loginfo("Moving right arm to home position...")
        self.move_to_position(self.HOME_POSITION, smooth=True)
        rospy.loginfo("Right arm controller initialized and ready")
        self.publish_joint_states()
    def clamp_angle(self, joint_name, angle):
        limits = self.JOINT_LIMITS[joint_name]
        return max(limits[0], min(limits[1], angle))
    def check_collision_avoidance(self, target_positions):
        safe_positions = target_positions.copy()
        if safe_positions['shoulder'] > 120:
            if safe_positions['elbow'] < 60:
                safe_positions['elbow'] = 60
                rospy.logwarn("Collision avoidance: Limited elbow angle (shoulder too high)")
        if safe_positions['elbow'] < 30:
            if safe_positions['arm'] < 45 or safe_positions['arm'] > 135:
                safe_positions['arm'] = 90
                rospy.logwarn("Collision avoidance: Limited arm angle (elbow too bent)")
        # Rule 3 removed because wrist joint no longer exists
        if safe_positions['arm'] > 150:
            if safe_positions['shoulder'] < 30:
                safe_positions['shoulder'] = 30
                rospy.logwarn("Collision avoidance: Limited shoulder angle (arm extended)")
        return safe_positions
    def move_joint(self, joint_name, target_angle, smooth=True):
        if joint_name not in self.JOINT_CHANNELS:
            rospy.logerr(f"Unknown joint: {joint_name}")
            return False
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
        safe_targets = {}
        for joint_name, angle in target_positions.items():
            if joint_name in self.JOINT_CHANNELS:
                safe_targets[joint_name] = self.clamp_angle(joint_name, angle)
            else:
                rospy.logwarn(f"Ignoring unknown joint: {joint_name}")
        safe_targets = self.check_collision_avoidance(safe_targets)
        
        # Enforce safe sequential movement order: Arm -> Shoulder -> Elbow
        motion_sequence = ['arm', 'shoulder', 'elbow', 'gripper']
        for joint_name in motion_sequence:
            if joint_name in safe_targets:
                self.move_joint(joint_name, safe_targets[joint_name], smooth=smooth)
        time.sleep(0.1)
    def joint_command_callback(self, msg):
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
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            joint_state = JointState()
            joint_state.header.stamp = rospy.Time.now()
            joint_state.name = [f"right_{name}" for name in self.JOINT_NAMES]
            joint_state.position = [self.current_positions[name] for name in self.JOINT_NAMES]
            joint_state.velocity = [0.0] * len(self.JOINT_NAMES)
            joint_state.effort = [0.0] * len(self.JOINT_NAMES)
            self.joint_state_pub.publish(joint_state)
            rate.sleep()
    def move_to_home(self):
        rospy.loginfo("Moving to home position...")
        self.move_to_position(self.HOME_POSITION, smooth=True)
    def shutdown(self):
        rospy.loginfo("Shutting down right arm controller...")
        self.move_to_home()
        rospy.loginfo("Right arm controller shut down")

def main():
    controller = None
    try:
        controller = RightArmController()
        rospy.on_shutdown(controller.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt received")
    except Exception as exc:
        rospy.logfatal(f"Right arm controller crashed: {exc}")
    finally:
        if controller is not None:
            controller.shutdown()


if __name__ == "__main__":
    main()

