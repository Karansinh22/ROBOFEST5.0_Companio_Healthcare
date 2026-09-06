#!/usr/bin/env python3
"""
ROS Noetic teleop node for dual arm control using keyboard.

Controls:
  < : Select LEFT arm
  > : Select RIGHT arm
  
  w : Move to TOP position
  s : Move FORWARD
  x : Move to BOTTOM position
  a : Move to OUTSIDE position
  d : Move to INSIDE position
  z : Move to MOVING/TRANSPORT position
  c : Move to HOME position
  
  q : Quit
"""

import sys
import tty
import termios
import rospy
from std_msgs.msg import Float64MultiArray


class DualArmTeleop:
    """Teleoperation controller for dual arm robot."""
    
    # Predefined positions for LEFT arm
    LEFT_ARM_POSITIONS = {
        'home': [70, 90, 5, 90, 0],
        'forward': [90, 90, 110, 90, 0],
        'outside': [90, 180, 110, 90, 0],
        'top': [0, 90, 110, 90, 0],
        'bottom': [179, 180, 110, 90, 0],
        'moving': [179, 180, 180, 90, 0]
    }
    
    # Predefined positions for RIGHT arm
    RIGHT_ARM_POSITIONS = {
        'home': [120, 90, 0, 95, 0],
        'forward': [90, 90, 90, 95, 0],
        'outside': [90, 0, 90, 90, 0],
        'top': [180, 90, 90, 90, 0],
        'bottom': [0, 0, 90, 90, 0],
        'moving': [0, 0, 180, 90, 0]
    }
    
    def __init__(self):
        """Initialize the teleop controller."""
        rospy.init_node('dual_arm_teleop', anonymous=False)
        
        # Publishers for both arms
        self.left_pub = rospy.Publisher(
            '/left_arm/joint_commands',
            Float64MultiArray,
            queue_size=10
        )
        
        self.right_pub = rospy.Publisher(
            '/right_arm/joint_commands',
            Float64MultiArray,
            queue_size=10
        )
        
        # Wait for publishers to connect
        rospy.sleep(0.5)
        
        # Currently selected arm
        self.selected_arm = 'left'
        
        rospy.loginfo("Dual Arm Teleop initialized")
        rospy.loginfo("Default arm: LEFT")
        self.print_help()
    
    def print_help(self):
        """Print control instructions."""
        print("\n" + "="*60)
        print("DUAL ARM TELEOP CONTROLS")
        print("="*60)
        print("\nARM SELECTION:")
        print("  <  : Select LEFT arm")
        print("  >  : Select RIGHT arm")
        print("\nMOVEMENT COMMANDS:")
        print("  w  : Move to TOP position")
        print("  s  : Move FORWARD")
        print("  x  : Move to BOTTOM position")
        print("  a  : Move to OUTSIDE position")
        print("  d  : Move to INSIDE position")
        print("  z  : Move to MOVING/TRANSPORT position")
        print("  c  : Move to HOME position")
        print("\nOTHER:")
        print("  q  : Quit")
        print("="*60)
        print(f"\nCurrently selected: {self.selected_arm.upper()} arm")
        print("Ready for commands...\n")
    
    def send_command(self, arm, position_name):
        """
        Send command to specified arm.
        
        Args:
            arm: 'left' or 'right'
            position_name: Name of the position to move to
        """
        if arm == 'left':
            if position_name not in self.LEFT_ARM_POSITIONS:
                rospy.logwarn(f"Unknown position: {position_name}")
                return
            
            position = self.LEFT_ARM_POSITIONS[position_name]
            msg = Float64MultiArray()
            msg.data = [float(x) for x in position]
            self.left_pub.publish(msg)
            rospy.loginfo(f"LEFT arm -> {position_name.upper()}: {position}")
            
        elif arm == 'right':
            if position_name not in self.RIGHT_ARM_POSITIONS:
                rospy.logwarn(f"Unknown position: {position_name}")
                return
            
            position = self.RIGHT_ARM_POSITIONS[position_name]
            msg = Float64MultiArray()
            msg.data = [float(x) for x in position]
            self.right_pub.publish(msg)
            rospy.loginfo(f"RIGHT arm -> {position_name.upper()}: {position}")
    
    def get_key(self):
        """Get single character from keyboard without waiting for Enter."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    
    def run(self):
        """Main teleop loop."""
        try:
            while not rospy.is_shutdown():
                key = self.get_key()
                
                # Arm selection
                if key == '<' or key == ',':
                    self.selected_arm = 'left'
                    print(f"\n>>> Selected: LEFT arm <<<")
                    
                elif key == '>' or key == '.':
                    self.selected_arm = 'right'
                    print(f"\n>>> Selected: RIGHT arm <<<")
                
                # Movement commands
                elif key == 'w' or key == 'W':
                    self.send_command(self.selected_arm, 'top')
                    
                elif key == 's' or key == 'S':
                    self.send_command(self.selected_arm, 'forward')
                    
                elif key == 'x' or key == 'X':
                    self.send_command(self.selected_arm, 'bottom')
                    
                elif key == 'a' or key == 'A':
                    self.send_command(self.selected_arm, 'outside')
                    
                elif key == 'd' or key == 'D':
                    self.send_command(self.selected_arm, 'inside')
                    
                elif key == 'z' or key == 'Z':
                    self.send_command(self.selected_arm, 'moving')
                    
                elif key == 'c' or key == 'C':
                    self.send_command(self.selected_arm, 'home')
                
                # Quit
                elif key == 'q' or key == 'Q':
                    print("\nQuitting teleop...")
                    break
                
                # Help
                elif key == 'h' or key == 'H':
                    self.print_help()
                
                # Invalid key
                elif key == '\x03':  # Ctrl+C
                    break
                else:
                    # Ignore other keys silently
                    pass
                    
        except Exception as e:
            rospy.logerr(f"Error in teleop: {e}")
        finally:
            rospy.loginfo("Dual Arm Teleop shutting down")


def main():
    """Main function."""
    try:
        teleop = DualArmTeleop()
        teleop.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received")
    except Exception as e:
        rospy.logerr(f"Teleop crashed: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()