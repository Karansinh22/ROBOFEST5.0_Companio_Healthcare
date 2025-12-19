#!/usr/bin/env python3

"""
Direct Keyboard Control for BumbleBot Motors
Controls motors via /lwheel_vtarget and /rwheel_vtarget topics

Motor Behavior:
- Left wheel: positive = CCW, negative = CW
- Right wheel: positive = CW, negative = CCW

Key Mappings:
    w - Forward (left CCW+, right CW+)
    s - Backward (left CW-, right CCW-)
    d - Turn Right (left CW-, right CW+)
    a - Turn Left (left CCW+, right CCW-)
    x - Stop (both 0)
    q - Quit
"""

import rospy
from std_msgs.msg import Float32
import sys
import termios
import tty

class DirectKeyboardControl:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('direct_keyboard_control', anonymous=True)
        
        # Publishers for left and right wheel velocity targets
        self.left_pub = rospy.Publisher('/lwheel_vtarget', Float32, queue_size=10)
        self.right_pub = rospy.Publisher('/rwheel_vtarget', Float32, queue_size=10)
        
        # Default speed (you can adjust this value)
        self.speed = 20.0
        
        # Current velocities
        self.left_vel = 0.0
        self.right_vel = 0.0
        
        # Publish rate
        self.rate = rospy.Rate(5)  # 5 Hz
        
        rospy.loginfo("Direct Keyboard Control Started")
        rospy.loginfo("Default speed: %.1f", self.speed)
        rospy.loginfo("\nControls:")
        rospy.loginfo("  w - Forward")
        rospy.loginfo("  s - Backward")
        rospy.loginfo("  a - Turn Left")
        rospy.loginfo("  d - Turn Right")
        rospy.loginfo("  x - Stop")
        rospy.loginfo("  q - Quit")
        rospy.loginfo("\nPress keys to control the robot...\n")
    
    def forward(self):
        """Move forward: left CCW (+), right CW (+)"""
        self.left_vel = self.speed
        self.right_vel = self.speed
        rospy.loginfo("FORWARD - Left: %.1f (CCW), Right: %.1f (CW)", self.left_vel, self.right_vel)
    
    def backward(self):
        """Move backward: left CW (-), right CCW (-)"""
        self.left_vel = -self.speed
        self.right_vel = -self.speed
        rospy.loginfo("BACKWARD - Left: %.1f (CW), Right: %.1f (CCW)", self.left_vel, self.right_vel)
    
    def turn_right(self):
        """Turn right: left CW (-), right CW (+)"""
        self.left_vel = -self.speed
        self.right_vel = self.speed
        rospy.loginfo("TURN RIGHT - Left: %.1f (CW), Right: %.1f (CW)", self.left_vel, self.right_vel)
    
    def turn_left(self):
        """Turn left: left CCW (+), right CCW (-)"""
        self.left_vel = self.speed
        self.right_vel = -self.speed
        rospy.loginfo("TURN LEFT - Left: %.1f (CCW), Right: %.1f (CCW)", self.left_vel, self.right_vel)
    
    def stop(self):
        """Stop both motors"""
        self.left_vel = 0.0
        self.right_vel = 0.0
        rospy.loginfo("STOP - Both motors stopped")
    
    def publish_velocities(self):
        """Publish current velocities to motor topics"""
        self.left_pub.publish(Float32(self.left_vel))
        self.right_pub.publish(Float32(self.right_vel))
    
    def get_key(self):
        """Get a single keypress from terminal"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key
    
    def run(self):
        """Main control loop"""
        try:
            while not rospy.is_shutdown():
                # Get keyboard input (non-blocking for this iteration)
                key = self.get_key()
                
                # Process key
                if key == 'w':
                    self.forward()
                elif key == 's':
                    self.backward()
                elif key == 'a':
                    self.turn_left()
                elif key == 'd':
                    self.turn_right()
                elif key == 'x':
                    self.stop()
                elif key == 'q':
                    rospy.loginfo("Quitting...")
                    self.stop()
                    self.publish_velocities()
                    break
                elif key == '\x03':  # Ctrl+C
                    break
                
                # Publish the current velocities
                self.publish_velocities()
                
                # Sleep to maintain publish rate
                self.rate.sleep()
                
        except Exception as e:
            rospy.logerr("Error: %s", str(e))
        finally:
            # Ensure motors are stopped on exit
            self.stop()
            self.publish_velocities()
            rospy.loginfo("Direct Keyboard Control Stopped")

if __name__ == '__main__':
    try:
        controller = DirectKeyboardControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass

