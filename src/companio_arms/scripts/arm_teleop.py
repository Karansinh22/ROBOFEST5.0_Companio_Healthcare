#!/usr/bin/env python3
"""
Dual Arm Teleop (incremental control with 5 degree steps)

Controls:
  1 : Select LEFT arm
  2 : Select RIGHT arm

After selecting an arm, control it with:
  w/s : shoulder up/down
  e/d : arm outside/inside
  r/f : elbow up/down
  r/f : elbow up/down
  y/h : gripper open/close

  c : Contract (rest posture, fits 30x30x30)
  x : Expand (working posture)
  q : Quit
  ? : Show help
"""

import sys
import termios
import tty
import rospy
from std_msgs.msg import Float64MultiArray, String

# Key bindings help text
HELP = """
=============================================================
Dual Arm Teleop (incremental, 5 degree steps)
=============================================================

Select arm:
  1 : Left arm
  2 : Right arm

After selecting, control the currently active arm with:
  w/s : shoulder up/down
  e/d : arm outside/inside
  r/f : elbow up/down
  r/f : elbow up/down
  y/h : gripper open/close

  c : Contract (rest posture, fits 30x30x30)
  x : Expand (working posture)
  q : Quit
  ? : Show this help
=============================================================
"""

INCREMENT = 5  # Degrees per step

left_state = [80, 85, 95, 160]
right_state = [90, 90, 90, 160]

# Joint limits for each arm
# Format: [min, max] for each joint
limit = {
    'left': [
        [0, 155],   # shoulder
        [10, 150],  # arm
        [15, 185],  # elbow
        [30, 175],  # gripper
    ],
    'right': [
        [0, 150],   # shoulder
        [0, 135],   # arm
        [5, 160],   # elbow
        [30, 175],  # gripper
    ]
}


def clamp(val, lo, hi):
    """Clamp value between min and max limits."""
    return max(lo, min(hi, val))


def get_key():
    """Read a single key press without requiring Enter."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        c = sys.stdin.read(1)
        if c == '\x03':  # Ctrl+C
            raise KeyboardInterrupt
        return c
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def print_state(arm, state):
    """Print current joint angles for the specified arm."""
    joint_names = ["shoulder", "arm", "elbow", "gripper"]
    status = ", ".join(f"{n}:{int(a)}°" for n, a in zip(joint_names, state))
    print(f"[{arm.upper()}] {status}")


def main():
    """Main teleop loop."""
    rospy.init_node('dual_arm_teleop')
    
    # Publishers for both arms
    left_pub = rospy.Publisher('/left_arm/joint_commands', Float64MultiArray, queue_size=1)
    posture_pub = rospy.Publisher('/posture/command', String, queue_size=1)
    right_pub = rospy.Publisher('/right_arm/joint_commands', Float64MultiArray, queue_size=1)
    
    # Wait for publishers to connect
    rospy.sleep(0.5)
    
    current_arm = 'left'
    print(HELP)
    print("Default: LEFT arm selected.")
    print_state("left", left_state)
    
    while not rospy.is_shutdown():
        print(f"\n[ACTIVE ARM: {current_arm.upper()}]   (press q to quit, ? for help)")
        key = get_key()
        
        # Get current state and limits for active arm
        state = left_state if current_arm == 'left' else right_state
        arm_lim = limit[current_arm]
        
        # Arm selection
        if key == '1':
            current_arm = 'left'
            print("\n>>> Switched to LEFT arm <<<")
            print_state("left", left_state)
            
        elif key == '2':
            current_arm = 'right'
            print("\n>>> Switched to RIGHT arm <<<")
            print_state("right", right_state)
        
        # Shoulder, Arm, and Elbow control (w/s, e/d, r/f)
        elif key in 'wsedrf':
            jmap = {
                'w': (0, -INCREMENT),  # shoulder up
                's': (0, +INCREMENT),  # shoulder down
                'e': (1, +INCREMENT),  # arm outside
                'd': (1, -INCREMENT),  # arm inside
                'r': (2, +INCREMENT),  # elbow up
                'f': (2, -INCREMENT),  # elbow down
            }
            idx, step = jmap[key]
            
            # Direction corrections for right arm (reversed geometry)
            if current_arm == 'right':
                if idx == 0:  # shoulder
                    step = -step
                if idx == 1:  # arm
                    step = -step
                # elbow is same for both arms
            
            state[idx] = clamp(state[idx] + step, arm_lim[idx][0], arm_lim[idx][1])
            print_state(current_arm, state)
        
            print_state(current_arm, state)
        
        # Gripper control (y/h)
        elif key in 'yh':
            idx = 3  # gripper joint index (was 4 when wrist existed)
            
            if key == 'y':
                # gripper open
                state[idx] = clamp(state[idx] - INCREMENT, arm_lim[idx][0], arm_lim[idx][1])
            else:  # key == 'h'
                # gripper close
                state[idx] = clamp(state[idx] + INCREMENT, arm_lim[idx][0], arm_lim[idx][1])
            
            print_state(current_arm, state)
        
        # Help
        elif key == 'c':
            posture_pub.publish(String(data='contract'))
            print('Contracting to rest posture ...')
        elif key == 'x':
            posture_pub.publish(String(data='expand'))
            print('Expanding to working posture ...')
        elif key == '?':
            print(HELP)
        
        # Quit
        elif key == 'q' or key == 'Q':
            print("\nExiting dual arm teleop...")
            break
        
        # Invalid key
        else:
            if key not in ['\r', '\n', ' ']:  # Ignore whitespace
                print(f"Invalid key: '{key}'. Press ? for help.")
        
        # Publish command to appropriate arm after every valid change
        if key in '12wsedrfyh':
            msg = Float64MultiArray()
            msg.data = list(state)
            
            if current_arm == 'left':
                left_pub.publish(msg)
            else:
                right_pub.publish(msg)
            
            rospy.sleep(0.02)  # Small delay for debouncing


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTeleop interrupted. Exiting cleanly.")
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()