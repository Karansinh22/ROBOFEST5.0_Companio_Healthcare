#!/usr/bin/env python3
"""
Tower Teleop (incremental control with 5 degree steps)

Controls:
  a/d : Pan left/right (Tower)
  w/s : Tilt up/down (Camera)
  h   : Reset to home position

  q : Quit
  ? : Show help
"""

import sys
import termios
import tty
import rospy
from std_msgs.msg import Float64MultiArray

HELP = """
=============================================================
Tower Teleop (incremental, 5 degree steps)
=============================================================

Controls:
  a/d : Pan left/right (Tower: 0-180)
  w/s : Tilt up/down (Camera: 0-180)
  h   : Reset to Home Position (Tower: 0, Camera: 90)

  q : Quit
  ? : Show this help
=============================================================
"""

INCREMENT = 5

# Initial positions [tower, camera]
HOME_STATE = [0, 90]
state = list(HOME_STATE)

# Format: [min, max] for each joint
limit = [
    [0, 180], # tower
    [0, 180]  # camera
]

def clamp(val, lo, hi):
    return max(lo, min(hi, val))

def get_key():
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

def print_state(state):
    joint_names = ["tower", "camera"]
    status = ", ".join(f"{n}:{int(a)}°" for n, a in zip(joint_names, state))
    sys.stdout.write(f"\r[TOWER] {status}                       \n")
    sys.stdout.flush()

def main():
    rospy.init_node('tower_teleop')
    pub = rospy.Publisher('/tower/joint_commands', Float64MultiArray, queue_size=1)
    rospy.sleep(0.5)
    
    print(HELP)
    print_state(state)
    
    while not rospy.is_shutdown():
        key = get_key()
        
        changed = False
        
        # Tower Pan
        if key == 'a':
            state[0] = clamp(state[0] - INCREMENT, limit[0][0], limit[0][1])
            changed = True
        elif key == 'd':
            state[0] = clamp(state[0] + INCREMENT, limit[0][0], limit[0][1])
            changed = True
            
        # Camera Tilt
        elif key == 'w':
            state[1] = clamp(state[1] - INCREMENT, limit[1][0], limit[1][1])
            changed = True
        elif key == 's':
            state[1] = clamp(state[1] + INCREMENT, limit[1][0], limit[1][1])
            changed = True
            
        # Home
        elif key == 'h':
            state[0], state[1] = HOME_STATE[0], HOME_STATE[1]
            changed = True
            
        # Help
        elif key == '?':
            print(HELP)
            
        # Quit
        elif key in ('q', 'Q'):
            print("\nExiting tower teleop...")
            break
            
        else:
            if key not in ['\r', '\n', ' ']:
                pass 
                
        if changed:
            print_state(state)
            msg = Float64MultiArray()
            msg.data = list(state)
            pub.publish(msg)
            rospy.sleep(0.02)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTeleop interrupted. Exiting cleanly.")
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"\nError: {e}")
