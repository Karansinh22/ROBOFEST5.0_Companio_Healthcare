#!/usr/bin/env python3
"""
Companio Right Arm Sequence Controller
- 'H': Executes a 5-stage motion sequence.
- 'D': Gripper cycle (Open -> 2s Wait -> Close).
"""

import rospy
import time
import sys
import termios
import tty
import select
from std_msgs.msg import Float64MultiArray, Bool

HELP = """
=============================================================
Companio Right Arm Sequence Controller
=============================================================
Press:
  H : Execute 5-stage shoulder/arm/elbow sequence
  D : Execute Gripper cycle (Open -> 2s -> Close)
  Q : Quit
=============================================================
"""

# Joint ordering: [shoulder, arm, elbow, gripper]
# Current limits for reference:
# Shoulder: [0, 150]
# Arm: [0, 150]
# Elbow: [5, 170]
# Gripper: [30, 170] (165 is closed)

H_SEQUENCE = [
    [100.0, 90.0, 110.0, 125.0], # Stage 1
    [25.0, 90.0, 110.0, 125.0],  # Stage 2
    [35.0, 90.0, 110.0, 125.0],  # Stage 3
    [35.0, 90.0, 110.0, 170.0],  # Stage 4
    [90.0, 90.0, 90.0, 170.0]    # Stage 5
]

# Sequence stage timing constants (matching controller speed)
INTERP_DELAY = 0.25 # seconds per 5 degrees
STEP_SIZE = 5.0
BUFFER = 1.0 # extra buffer for ROS/Processing overhead

def calculate_wait_time(target, current):
    """Estimate time for controller to finish sequential move."""
    total_delta = sum(abs(t - c) for t, c in zip(target, current))
    if total_delta < 1.0:
        return 0.5 # Minimum wait
    return (total_delta / STEP_SIZE) * INTERP_DELAY + BUFFER

auto_pick_requested = False
last_vision_state = False
other_arm_moving = False

def status_callback(msg):
    global other_arm_moving
    other_arm_moving = msg.data

def vision_callback(msg):
    global auto_pick_requested, last_vision_state
    if msg.data and not last_vision_state:
        # Transition from False to True
        auto_pick_requested = True
    last_vision_state = msg.data

def get_key(timeout=0.1):
    """Read a single key press without requiring Enter (non-blocking)."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if r:
            c = sys.stdin.read(1)
            if c == '\x03':  # Ctrl+C
                raise KeyboardInterrupt
            return c
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def main():
    rospy.init_node('right_arm_sequence_controller')
    pub = rospy.Publisher('/right_arm/joint_commands', Float64MultiArray, queue_size=1)
    
    # Subscribe to vision system
    rospy.Subscriber('/gripper_vision/right/target_detected', Bool, vision_callback)
    
    # Mutual exclusion topics
    status_pub = rospy.Publisher('/right_arm/is_moving', Bool, queue_size=1, latch=True)
    status_pub.publish(False)
    rospy.Subscriber('/left_arm/is_moving', Bool, status_callback)
    
    # Initialize tracking - starting from home
    current_pos = [90.0, 90.0, 90.0, 160.0]
    last_pick_time = 0.0
    
    rospy.sleep(0.5) # Wait for pub to connect
    print(HELP)

    prompt_printed = False

    while not rospy.is_shutdown():
        if not prompt_printed:
            print(f"\nCurrent Position: {current_pos}")
            print("Waiting for key (H, D, Q) or Vision Trigger...")
            prompt_printed = True

        key = get_key(0.1)
        if key:
            key = key.upper()

        global auto_pick_requested
        if auto_pick_requested:
            if time.time() - last_pick_time > 15.0:
                print("\n>>> AUTO-PICK TRIGGERED BY VISION SYSTEM <<<")
                key = 'H'
                last_pick_time = time.time()
            auto_pick_requested = False

        if key:
            prompt_printed = False # Reset prompt for next iteration

        if key == 'H':
            if other_arm_moving:
                print(">>> WARNING: Cannot move. Left arm is currently active. <<<")
                prompt_printed = False
                continue

            print(">>> Executing 5-stage sequence 'H'...")
            status_pub.publish(True)
            for i, stage in enumerate(H_SEQUENCE):
                wait = calculate_wait_time(stage, current_pos)
                print(f"  Stage {i+1}: {stage} (Waiting {wait:.2f}s)")
                
                msg = Float64MultiArray()
                msg.data = stage
                pub.publish(msg)
                
                rospy.sleep(wait)
                current_pos = list(stage)
                
            auto_pick_requested = False # Clear any triggers queued during movement
            status_pub.publish(False)

        elif key == 'D':
            if other_arm_moving:
                print(">>> WARNING: Cannot move. Left arm is currently active. <<<")
                prompt_printed = False
                continue

            print(">>> Executing Gripper cycle 'D'...")
            # Step 1: Open gripper to 125
            open_pos = [current_pos[0], current_pos[1], current_pos[2], 125.0]
            wait_open = calculate_wait_time(open_pos, current_pos)
            
            print(f"  Opening gripper to 125... (Wait {wait_open:.2f}s)")
            msg = Float64MultiArray()
            msg.data = open_pos
            pub.publish(msg)
            current_pos = list(open_pos)
            
            rospy.sleep(wait_open + 2.0) # wait for move + 2s requested
            
            # Step 2: Close gripper to 160
            close_pos = [current_pos[0], current_pos[1], current_pos[2], 160.0]
            wait_close = calculate_wait_time(close_pos, current_pos)
            print(f"  Closing gripper to 160... (Wait {wait_close:.2f}s)")
            msg.data = close_pos
            pub.publish(msg)
            current_pos = list(close_pos)
            rospy.sleep(wait_close)
            status_pub.publish(False)

        elif key == 'Q':
            print("Exiting...")
            break
        
        elif key is not None:
            print(f"Ignored key: {key}")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
