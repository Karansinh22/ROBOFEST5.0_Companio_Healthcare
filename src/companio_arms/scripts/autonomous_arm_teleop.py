#!/usr/bin/env python3
"""
Autonomous Right Arm Teleop (Standalone, no external imports)

Prompts for (X,Y,Z) targets, computes analytic IK locally,
shows the predicted angles, then publishes to /right_arm/move_to_xyz
for right_arm_ik_controller.py to execute with sequential 5° steps.

No reliance on importing other ROS script files.
"""

import math
import sys
import rospy
from std_msgs.msg import Float64MultiArray

# ── Arm geometry (must match right_arm_ik_controller.py) ──────────────────────
ORIGIN_X  = 10.5   # Right arm X offset from robot centre (cm)
ORIGIN_Y  = 0.0
ORIGIN_Z  = 0.0
L2        = 9.5    # Shoulder → Elbow
L3        = 4.5    # Elbow → Wrist/Gripper base
L4        = 11.5   # Gripper tip
MAX_REACH = 29.0

LIMITS = {
    'shoulder': [40,  150],
    'arm':      [75,  150],
    'elbow':    [5,   160],
    'gripper':  [105, 160],
}

# ── No-Go Zones ───────────────────────────────────────────────────────────────
NO_GO_ZONES = [
    (-13,  13,  -5,  5,  -21.5, -13.5, "Base Box"),
    ( -7,   7,  -4,  4,  -21.5,   8.5, "Tower Column"),
    (-5.5, 5.5,  8, 12,   -1.5,   2.5, "Camera Zone"),
]


def check_collision(x, y, z):
    if x <= 0:
        return True, "Center Line (X <= 0)"
    for x0, x1, y0, y1, z0, z1, name in NO_GO_ZONES:
        if x0 <= x <= x1 and y0 <= y <= y1 and z0 <= z <= z1:
            return True, name
    return False, ""


def compute_ik(tx, ty, tz):
    """
    Analytic IK — returns dict with shoulder/arm/elbow or None.
    Convention: shoulder=90 → horizontal, 0=up, >90=down.
                arm=90 → straight ahead. >90 = more outward.
                elbow=90 → straight.
    """
    x = tx - ORIGIN_X
    y = ty - ORIGIN_Y
    z = tz - ORIGIN_Z

    dist_2d = math.sqrt(x ** 2 + y ** 2)
    dist_3d = math.sqrt(dist_2d ** 2 + z ** 2)

    if dist_3d > MAX_REACH:
        return None

    # Arm (yaw)
    arm_rad = math.atan2(x, y)
    arm_deg = 90 + math.degrees(arm_rad)

    # Shoulder elevation
    elevation_rad = math.atan2(z, dist_2d)
    shoulder_deg  = 90 - math.degrees(elevation_rad)

    # Elbow
    dist_ratio = dist_3d / MAX_REACH
    if dist_ratio > 0.95:
        elbow_deg = 90
    else:
        link_a = L2
        link_b = L3 + L4
        try:
            cA = (link_a**2 + link_b**2 - dist_3d**2) / (2*link_a*link_b)
            cA = max(-1.0, min(1.0, cA))
            interior_rad = math.acos(cA)
            elbow_deg    = 90 + (180 - math.degrees(interior_rad))
            # Shoulder compensation
            alpha        = math.acos(max(-1.0, min(1.0,
                           (link_a**2 + dist_3d**2 - link_b**2) / (2*link_a*dist_3d))))
            shoulder_deg += math.degrees(alpha)
        except ValueError:
            return None

    # Clamp to limits
    def clamp(v, lo, hi):
        return max(lo, min(hi, v))

    return {
        'shoulder': clamp(shoulder_deg, *LIMITS['shoulder']),
        'arm':      clamp(arm_deg,      *LIMITS['arm']),
        'elbow':    clamp(elbow_deg,    *LIMITS['elbow']),
    }


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    rospy.init_node('autonomous_arm_teleop', anonymous=True)
    pub = rospy.Publisher('/right_arm/move_to_xyz', Float64MultiArray, queue_size=1)
    rospy.sleep(0.5)

    print("="*60)
    print("  Autonomous Right Arm IK Teleop")
    print("  Origin (0,0,0) = midpoint between shoulders")
    print("  Right arm kinematic origin at (10.5, 0, 0)")
    print("  Enter 'q' to quit.")
    print("="*60 + "\n")

    while not rospy.is_shutdown():
        try:
            raw = input("Target (X Y Z) > ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting.")
            break

        if raw.lower() in ('q', 'quit', 'exit'):
            print("Exiting.")
            break

        parts = raw.split()
        if len(parts) != 3:
            print("  ✗  Provide exactly 3 numbers: X Y Z")
            continue
        try:
            tx, ty, tz = float(parts[0]), float(parts[1]), float(parts[2])
        except ValueError:
            print("  ✗  Non-numeric input.")
            continue

        # Safety check target and approach point (7cm above)
        blocked, reason = check_collision(tx, ty, tz)
        blocked_a, reason_a = check_collision(tx, ty, tz + 7.0)
        
        if blocked:
            print(f"  ✗  TARGET BLOCKED — '{reason}'")
            continue
        if blocked_a:
            print(f"  ✗  APPROACH (+7cm) BLOCKED — '{reason_a}'")
            continue

        # IK
        angles = compute_ik(tx, ty, tz)
        if angles is None:
            print("  ✗  Target unreachable (beyond arm range or IK math error).")
            continue

        # Show prediction
        print(f"\n  [PREDICTION]")
        print(f"    Shoulder : {angles['shoulder']:.1f}°")
        print(f"    Arm      : {angles['arm']:.1f}°")
        print(f"    Elbow    : {angles['elbow']:.1f}°")

        try:
            confirm = input("\n  Execute? [y/N] > ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if confirm != 'y':
            print("  Motion aborted.")
            continue

        msg = Float64MultiArray()
        msg.data = [tx, ty, tz]
        pub.publish(msg)
        print(f"  Sent ({tx}, {ty}, {tz}) → right_arm_ik_controller")


if __name__ == '__main__':
    main()
