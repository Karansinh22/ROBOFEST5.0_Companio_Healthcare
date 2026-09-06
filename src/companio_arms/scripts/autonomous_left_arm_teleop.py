#!/usr/bin/env python3
"""
Autonomous Left Arm Teleop (Coordinate Input)

Provides a clean terminal interface to send (X,Y,Z) coordinates
to the Left Arm IK Controller node.
"""

import sys
import rospy
from std_msgs.msg import Float64MultiArray
from left_arm_ik_controller import LeftArmIK

def main():
    rospy.init_node('autonomous_left_arm_teleop', anonymous=True)
    ik_pub = rospy.Publisher('/left_arm/move_to_xyz', Float64MultiArray, queue_size=1)
    
    rospy.sleep(0.5)
    
    print("=============================================================")
    print("Autonomous LEFT Arm IK Coordinate Controller")
    print("Robot Origin (0,0,0) is midway between shoulders at Z=-21.5 (base)")
    print("=============================================================")
    print("\nEnter coordinates as: X Y Z (e.g., -10.5 15 5)")
    print("Type 'q' or 'quit' to exit.")
    
    while not rospy.is_shutdown():
        try:
            user_input = input("\nTarget (X Y Z) > ").strip().lower()
            
            if user_input in ['q', 'quit', 'exit']:
                print("Exiting...")
                break
                
            parts = user_input.split()
            if len(parts) != 3:
                print("Error: Please provide exactly 3 numbers separated by spaces.")
                continue
                
            x, y, z = [float(p) for p in parts]
            
            collision, reason = LeftArmIK.check_collision(x, y, z)
            if collision:
                print(f"MOTION BLOCKED: {reason}")
                continue
                
            angles = LeftArmIK.compute_ik(x, y, z)
            if angles is None:
                print("Target unreachable based on kinematics.")
                continue
                
            print(f"\n[PREDICTION] Final Left Motor Angles:")
            print(f"  Shoulder : {angles['shoulder']:.1f} degrees")
            print(f"  Arm      : {angles['arm']:.1f} degrees")
            print(f"  Elbow    : {angles['elbow']:.1f} degrees")
            
            confirm = input("\nExecute this sequential motion? [y/N] > ").strip().lower()
            if confirm != 'y':
                print("Motion aborted by user. Please enter a new target.")
                continue
            
            msg = Float64MultiArray()
            msg.data = [x, y, z]
            ik_pub.publish(msg)
            
            print(f"Sent command to move to: ({x}, {y}, {z})")
            print("Waiting for Left IK controller to execute sequential motion...")
            
        except ValueError:
            print("Error: Non-numeric input detected. Please enter numbers only.")
        except KeyboardInterrupt:
            print("\nExiting...")
            break

if __name__ == '__main__':
    main()
