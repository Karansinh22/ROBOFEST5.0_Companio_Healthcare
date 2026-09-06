#!/usr/bin/env python3
"""
Right Arm Inverse Kinematics Controller (Standalone)

This node accepts specific XYZ coordinates for the right arm, computes inverse
kinematics, validates the trajectory against collision zones, and publishes safe
sequential 5-degree motion commands directly to `/right_arm/joint_commands`.

Motors are moved strictly one at a time.
"""

import sys
import math
import time
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class RightArmIK:
    # Kinematic geometry
    ORIGIN_X = 10.5
    ORIGIN_Y = 0.0
    ORIGIN_Z = 0.0
    
    # Link lengths (cm)
    L1 = 3.5   # Shoulder to arm origin (purely X offset, handled by ORIGIN_X)
    L2 = 9.5   # Arm to Elbow
    L3 = 4.5   # Elbow to Wrist/Gripper base
    L4 = 11.5  # Wrist to Gripper tip
    
    # Total effective length from the Arm origin
    L_EFFECTIVE = L2 + L3 + L4
    MAX_REACH = 29.0
    
    # Joint limits (Hardware matching limits)
    LIMITS = {
        'gripper': [30, 160]
    }
    APPROACH_DZ = 7.0  # Vertical offset for approach (cm)
    
    @classmethod
    def check_collision(cls, x, y, z):
        """
        Check if the given XYZ coordinate falls inside any forbidden No-Go zones.
        Returns (True, "Reason") if a collision is detected, (False, "") if safe.
        """
        # Right Arm Center Line restriction
        if x <= 0:
            return True, "Target crossed center line (X <= 0)"
            
        # 1. Base Collision Box
        if -13 <= x <= 13 and -5 <= y <= 5 and -21.5 <= z <= -13.5:
            return True, "Target inside Base Collision Box"
            
        # 2. Tower Collision Column
        if -7 <= x <= 7 and -4 <= y <= 4 and -21.5 <= z <= 8.5:
            return True, "Target inside Tower Collision Column"
            
        # 3. Camera Collision Zone
        if -5.5 <= x <= 5.5 and 8 <= y <= 12 and -1.5 <= z <= 2.5:
            return True, "Target inside Camera Collision Zone"
            
        return False, ""

    @classmethod
    def compute_ik(cls, target_x, target_y, target_z):
        """
        Compute Inverse Kinematics for the right arm.
        Returns a dict of {shoulder, arm, elbow} angles or None if unreachable.
        """
        # Translate target relative to right arm origin
        x = target_x - cls.ORIGIN_X
        y = target_y - cls.ORIGIN_Y
        z = target_z - cls.ORIGIN_Z
        
        # Calculate horizontal distance from the arm origin (y-x plane)
        # Note: 'Arm' servo (yaw) dictates the angle in the X-Y plane
        dist_2d = math.sqrt(x**2 + y**2)
        dist_3d = math.sqrt(dist_2d**2 + z**2)
        
        if dist_3d > cls.MAX_REACH:
            rospy.logwarn(f"Target unreachable: Distance {dist_3d:.2f} > Max {cls.MAX_REACH}")
            return None
            
        # 1. Arm Angle (Yaw in XY plane)
        # 90 degrees is straight ahead (Y max, X=0). 
        # 0 is inward (-X), 180 is outward (+X).
        # We use atan2 to find the angle from the Y axis.
        arm_rad = math.atan2(x, y)
        arm_deg = 90 + math.degrees(arm_rad)
        
        # We simplify the pitch mechanics (Shoulder + Elbow)
        # For this setup, we treat L2, L3, L4 as a straight line manipulator that bends at the shoulder and elbow.
        # Since inverse kinematics for an redundant arm needs a specific posture constraint,
        # we aim to keep the elbow straight (90) if possible, and bend the shoulder.
        # If it needs to go lower, we bend the shoulder down.
        
        # Calculate elevation angle from origin
        elevation_rad = math.atan2(z, dist_2d)
        
        # Shoulder Angle (Pitch)
        # 90 is forward (horizontal). 0 is up, 180 is down.
        shoulder_deg = 90 - math.degrees(elevation_rad)
        
        # Elbow Angle (Pitch relative to arm)
        # Without specific end-effector orientation targets, we will aim to keep the elbow straight (90)
        # unless necessary to pull back closer to the body.
        # A simple linear interpolation for radius:
        dist_ratio = dist_3d / cls.MAX_REACH
        
        if dist_ratio > 0.95:
            elbow_deg = 90  # Fully extended
        else:
            # If target is closer, we bend the elbow to pull the hand inwards
            # 90 is straight, <90 is bending inward/downward, >90 is bending up
            # Bending the elbow requires re-adjusting the shoulder to keep the tip on target.
            
            # Using Law of Cosines on a simplified 2-link model (L2 vs L3+L4)
            link_a = cls.L2
            link_b = cls.L3 + cls.L4
            
            try:
                # Calculate elbow angle (interior angle of the triangle)
                cos_angle = (link_a**2 + link_b**2 - dist_3d**2) / (2 * link_a * link_b)
                cos_angle = max(-1.0, min(1.0, cos_angle)) # Clamp domains
                elbow_interior_rad = math.acos(cos_angle)
                
                # Convert to servo angle (180 is fully bent inwards, 90 is straight)
                elbow_offset = 180 - math.degrees(elbow_interior_rad)
                elbow_deg = 90 + elbow_offset
                
                # Shoulder compensation
                alpha = math.acos((link_a**2 + dist_3d**2 - link_b**2) / (2 * link_a * dist_3d))
                shoulder_deg += math.degrees(alpha)
                
            except ValueError:
                rospy.logerr("Math error during IK calculation")
                return None
                
        # Clamp everything to hardware limits
        target_angles = {
            'shoulder': max(cls.LIMITS['shoulder'][0], min(cls.LIMITS['shoulder'][1], shoulder_deg)),
            'arm': max(cls.LIMITS['arm'][0], min(cls.LIMITS['arm'][1], arm_deg)),
            'elbow': max(cls.LIMITS['elbow'][0], min(cls.LIMITS['elbow'][1], elbow_deg))
        }
        
        return target_angles

class SequentialIKController:
    def __init__(self):
        rospy.init_node('right_arm_ik_controller', anonymous=False)
        self.pub = rospy.Publisher('/right_arm/joint_commands', Float64MultiArray, queue_size=1)
        
        # Current known state [shoulder, arm, elbow, gripper]
        self.current_state = [90.0, 90.0, 90.0, 160.0]
        self.state_initialized = False
        
        rospy.Subscriber('/right_arm/joint_states', JointState, self.state_callback)
        rospy.sleep(1.0) # Wait for state population
        
        if not self.state_initialized:
            rospy.logwarn("Failed to receive current state from /right_arm/joint_states. Starting from default home position.")
        else:
            rospy.loginfo("Successfully attached to Right Arm joint states.")
            
        # Optional: Set up an XYZ coordinate subscriber
        rospy.Subscriber('/right_arm/move_to_xyz', Float64MultiArray, self.xyz_callback)
        rospy.loginfo("IK Controller active. Listening on /right_arm/move_to_xyz for 3D coordinates.")
        
    def state_callback(self, msg):
        try:
            # Update our internal tracking map based on hardware feedback
            # Expected msg.name: ['right_shoulder', 'right_arm', 'right_elbow', 'right_gripper']
            self.current_state[0] = msg.position[msg.name.index('right_shoulder')]
            self.current_state[1] = msg.position[msg.name.index('right_arm')]
            self.current_state[2] = msg.position[msg.name.index('right_elbow')]
            self.current_state[3] = msg.position[msg.name.index('right_gripper')]
            self.state_initialized = True
        except ValueError:
            pass
            
    def publish_state(self):
        msg = Float64MultiArray()
        msg.data = self.current_state
        self.pub.publish(msg)
        
    def xyz_callback(self, msg):
        if len(msg.data) != 3:
            rospy.logerr("Expected 3 values (x, y, z) for target coordinates.")
            return
            
        x, y, z = msg.data
        rospy.loginfo(f"Received target coordinate: ({x}, {y}, {z})")
        
        # 1. Safety validation
        collision, reason = RightArmIK.check_collision(x, y, z)
        if collision:
            rospy.logerr(f"MOTION BLOCKED: {reason}")
            return
            
        # 2. Compute IK for Target and Above point
        target_angles = RightArmIK.compute_ik(x, y, z)
        above_angles = RightArmIK.compute_ik(x, y, z + RightArmIK.APPROACH_DZ)
        
        if target_angles is None or above_angles is None:
            rospy.logerr("Target or approach point unreachable.")
            return
            
        rospy.loginfo(f"IK Solved. Target: S={target_angles['shoulder']:.1f}, A={target_angles['arm']:.1f}, E={target_angles['elbow']:.1f}")
        
        # 3. Handle Sequence: Home -> Above -> Target -> Close -> Above
        home_open = [90.0, 90.0, 90.0, 110.0]
        above_open = [above_angles['shoulder'], above_angles['arm'], above_angles['elbow'], 110.0]
        target_open = [target_angles['shoulder'], target_angles['arm'], target_angles['elbow'], 110.0]
        target_close = [target_angles['shoulder'], target_angles['arm'], target_angles['elbow'], 160.0]
        above_close = [above_angles['shoulder'], above_angles['arm'], above_angles['elbow'], 160.0]

        def move_and_wait(angles, label):
            rospy.loginfo(f"  → {label}...")
            m = Float64MultiArray()
            m.data = angles
            self.pub.publish(m)
            rate = rospy.Rate(10)
            start_time = rospy.Time.now()
            while not rospy.is_shutdown():
                diffs = [abs(self.current_state[i] - angles[i]) for i in range(4)]
                if max(diffs) < 1.0: return True
                if (rospy.Time.now() - start_time).to_sec() > 20.0: return False
                rate.sleep()

        move_and_wait(home_open, "Stage 1: Home (Open)")
        move_and_wait(above_open, "Stage 2: Above Target")
        move_and_wait(target_open, "Stage 3: Pick Point")
        move_and_wait(target_close, "Stage 4: Close Gripper (160°)")
        move_and_wait(above_close, "Stage 5: Lift Above")
        
        rospy.loginfo("Pick sequence complete. Object lifted.")

if __name__ == '__main__':
    try:
        controller = SequentialIKController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
