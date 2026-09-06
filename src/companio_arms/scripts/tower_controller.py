#!/usr/bin/env python3
"""
ROS Noetic node for controlling the tower of a dual-arm hospital corridor robot.

Hardware: PCA9685 16-channel PWM servo driver
Tower Joints:
  - Tower Motor[14]: 0 to 180 degrees (pan)
  - Camera Motor[15]: 0 to 180 degrees (tilt)

Features:
  - Safety limits 
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


class TowerController:
    JOINT_CHANNELS = {
        'tower': 14,
        'camera': 15
    }
    
    JOINT_NAMES = ['tower', 'camera']
    
    JOINT_LIMITS = {
        'tower': [0, 180],
        'camera': [0, 180]
    }
    
    HOME_POSITION = {
        'tower': 0,
        'camera': 90
    }
    
    def __init__(self):
        rospy.init_node('tower_controller', anonymous=False)
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
            '/tower/joint_states',
            JointState,
            queue_size=10)
            
        rospy.Subscriber(
            '/tower/joint_commands',
            Float64MultiArray,
            self.joint_command_callback,
            queue_size=1)
            
        rospy.loginfo("Moving tower to home position...")
        self.move_to_position(self.HOME_POSITION, smooth=True)
        rospy.loginfo("Tower controller initialized and ready")
        self.publish_joint_states()
        
    def clamp_angle(self, joint_name, angle):
        limits = self.JOINT_LIMITS[joint_name]
        return max(limits[0], min(limits[1], angle))
        
    def check_collision_avoidance(self, target_positions):
        safe_positions = target_positions.copy()
        # Add tower-specific collision avoidance logic here if needed
        return safe_positions
        
    def move_joint(self, joint_name, target_angle, smooth=True):
        if joint_name not in self.JOINT_CHANNELS:
            rospy.logerr(f"Unknown joint: {joint_name}")
            return False
            
        target_angle = self.clamp_angle(joint_name, target_angle)
        channel = self.JOINT_CHANNELS[joint_name]
        current_angle = self.current_positions[joint_name]
        
        if smooth and abs(target_angle - current_angle) > 1:
            steps = max(self.interpolation_steps, int(abs(target_angle - current_angle) / 5))
            for i in range(steps + 1):
                angle = current_angle + (target_angle - current_angle) * (i / steps)
                self.pca9685.set_angle(channel, angle)
                self.current_positions[joint_name] = angle
                time.sleep(self.move_speed / steps)
        else:
            self.pca9685.set_angle(channel, target_angle)
            self.current_positions[joint_name] = target_angle
            if smooth:
                time.sleep(self.move_speed * abs(target_angle - current_angle) / 90.0)
                
        return True
        
    def move_to_position(self, target_positions, smooth=True):
        safe_targets = {}
        for joint_name, angle in target_positions.items():
            if joint_name in self.JOINT_CHANNELS:
                safe_targets[joint_name] = self.clamp_angle(joint_name, angle)
            else:
                rospy.logwarn(f"Ignoring unknown joint: {joint_name}")
                
        safe_targets = self.check_collision_avoidance(safe_targets)
        for joint_name in self.JOINT_NAMES:
            if joint_name in safe_targets:
                self.move_joint(joint_name, safe_targets[joint_name], smooth=smooth)
        time.sleep(0.1)
        
    def joint_command_callback(self, msg):
        if len(msg.data) != 2:
            rospy.logerr(f"Invalid joint command: expected 2 values, got {len(msg.data)}")
            return
            
        target_positions = {
            'tower': float(msg.data[0]),
            'camera': float(msg.data[1])
        }
        
        rospy.loginfo(f"Received joint command: {target_positions}")
        self.move_to_position(target_positions, smooth=True)
        
    def publish_joint_states(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            joint_state = JointState()
            joint_state.header.stamp = rospy.Time.now()
            # Publish joint names as tower_tower and tower_camera
            joint_state.name = [f"tower_{name}" for name in self.JOINT_NAMES]
            joint_state.position = [self.current_positions[name] for name in self.JOINT_NAMES]
            joint_state.velocity = [0.0] * len(self.JOINT_NAMES)
            joint_state.effort = [0.0] * len(self.JOINT_NAMES)
            
            self.joint_state_pub.publish(joint_state)
            rate.sleep()
            
    def move_to_home(self):
        rospy.loginfo("Moving to home position...")
        self.move_to_position(self.HOME_POSITION, smooth=True)
        
    def shutdown(self):
        rospy.loginfo("Shutting down tower controller...")
        self.move_to_home()
        rospy.loginfo("Tower controller shut down")


def main():
    controller = None
    try:
        controller = TowerController()
        rospy.on_shutdown(controller.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt received")
    except Exception as exc:
        rospy.logfatal(f"Tower controller crashed: {exc}")
    finally:
        if controller is not None:
            controller.shutdown()


if __name__ == "__main__":
    main()
