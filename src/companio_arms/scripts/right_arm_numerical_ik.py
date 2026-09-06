#!/usr/bin/env python3
"""
Right Arm Numerical IK Solver — Standalone ROS Node
==========================================================
Brute-force numeric search instead of analytic IK.

CORRECTED FK MODEL (matches analytic right_arm_ik_controller.py):

  ORIGIN_X = 10.5 cm  (right arm kinematic origin from robot centre)

  shoulder_deg → elevation control
      90° = horizontal, 0° = fully up, 150° = fully down
      elev_s = 90 - shoulder_deg  (degrees above horizontal)

  elbow_deg → bend control  
      90° = straight, <90° = bending inward
      elev_e = 90 - elbow_deg  (bend deviation in elevation)

  Combined reach in elevation plane:
      reach_h = L1*cos(elev_s) + L2*cos(elev_s + elev_e)
      reach_v = L1*sin(elev_s) + L2*sin(elev_s + elev_e)

  arm_deg → yaw control
      90° = straight ahead (pure Y), >90 = more X outward
      yaw = arm_deg - 90  (degrees offset from straight)

  Global position:
      gx = ORIGIN_X + sin(yaw_rad) * reach_h
      gy = cos(yaw_rad) * reach_h
      gz = reach_v

Limits:
  shoulder: 40° – 150°
  arm:      75° – 150°
  elbow:     5° – 160°
  gripper: 105° – 160°

Motion rules:
  - 5° increments per step
  - 0.5 s delay between steps
  - One motor at a time
  - Always ask user before moving
"""

import math
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

# ── Geometry ──────────────────────────────────────────────────────────────────
L1         = 9.5    # Shoulder → Elbow (cm)
L2         = 16.0   # Elbow → Gripper tip (cm)
ORIGIN_X   = 10.5   # Right arm X offset from robot centre (cm)

# ── Joint Limits ──────────────────────────────────────────────────────────────
SHOULDER_MIN, SHOULDER_MAX = 40,  150
ARM_MIN,      ARM_MAX      = 75,  150
ELBOW_MIN,    ELBOW_MAX    = 5,   160
GRIPPER_MIN,  GRIPPER_MAX  = 105, 160

# ── Search / Motion Parameters ────────────────────────────────────────────────
SEARCH_STEP  = 5    # Search grid resolution (degrees)
MOTOR_STEP   = 5    # Movement step size (degrees)
MOTOR_DELAY  = 0.5  # Delay between motor steps (seconds)
WARN_ERR_CM  = 2.0  # Warn user when error exceeds this value
APPROACH_DZ  = 7.0  # Vertical offset for approaching objects (cm)

# ── No-Go Zones ───────────────────────────────────────────────────────────────
NO_GO_ZONES = [
    (-13,  13,  -5,  5,  -21.5, -13.5, "Base Box"),
    ( -7,   7,  -4,  4,  -21.5,   8.5, "Tower Column"),
    (-5.5, 5.5,  8, 12,   -1.5,   2.5, "Camera Zone"),
]


# ── Forward Kinematics ────────────────────────────────────────────────────────
def fk(shoulder_deg, arm_deg, elbow_deg):
    """Return global (x, y, z) for the given servo angles."""
    es = math.radians(90.0 - shoulder_deg)   # elevation from horizontal
    ee = math.radians(elbow_deg - 90.0)      # elbow bend contribution (180=up, 0=down)

    rh = L1 * math.cos(es) + L2 * math.cos(es + ee)   # horizontal reach
    rv = L1 * math.sin(es) + L2 * math.sin(es + ee)   # vertical reach

    yaw = math.radians(arm_deg - 90.0)
    return (
        ORIGIN_X + math.sin(yaw) * rh,   # gx
        math.cos(yaw) * rh,               # gy
        rv                                # gz
    )


# ── Collision Check ───────────────────────────────────────────────────────────
def in_no_go(x, y, z):
    """Returns (True, name) if point is in a forbidden zone."""
    for x0, x1, y0, y1, z0, z1, name in NO_GO_ZONES:
        if x0 <= x <= x1 and y0 <= y <= y1 and z0 <= z <= z1:
            return True, name
    if x <= 0:
        return True, "Center Line (X ≤ 0)"
    return False, ""


# ── IK Search ────────────────────────────────────────────────────────────────
def ik_search(tx, ty, tz):
    """
    Grid search over all joint combinations.
    Returns (best_angles_tuple, best_error_cm).
    best_angles is None if every combination is inside a no-go zone.
    """
    best_err = float('inf')
    best     = None

    srange = range(SHOULDER_MIN, SHOULDER_MAX + 1, SEARCH_STEP)
    arange = range(ARM_MIN,      ARM_MAX      + 1, SEARCH_STEP)
    erange = range(ELBOW_MIN,    ELBOW_MAX    + 1, SEARCH_STEP)
    total  = len(srange) * len(arange) * len(erange)
    rospy.loginfo(f"Searching {total} combinations ({SEARCH_STEP}° step)…")

    for s in srange:
        for a in arange:
            for e in erange:
                gx, gy, gz = fk(s, a, e)
                err = math.sqrt((gx-tx)**2 + (gy-ty)**2 + (gz-tz)**2)
                if err < best_err:
                    blocked, _ = in_no_go(gx, gy, gz)
                    if not blocked:
                        best_err = err
                        best     = (s, a, e)

    return best, best_err


# ── ROS Node ─────────────────────────────────────────────────────────────────
class NumericalIKNode:
    def __init__(self):
        rospy.init_node('right_arm_numerical_ik', anonymous=False)
        self.pub   = rospy.Publisher('/right_arm/joint_commands',
                                     Float64MultiArray, queue_size=1)
        self.state = [90.0, 90.0, 90.0, 160.0]   # shoulder,arm,elbow,gripper
        self.synced = False

        rospy.Subscriber('/right_arm/joint_states', JointState, self._state_cb)
        rospy.sleep(1.0)
        if not self.synced:
            rospy.logwarn("No joint state feedback — starting from home [90,90,90,160].")
        else:
            rospy.loginfo("Synced with right_arm_controller.")
        rospy.loginfo("Numerical IK node ready.")

    def _state_cb(self, msg):
        try:
            i = msg.name.index
            self.state[0] = msg.position[i('right_shoulder')]
            self.state[1] = msg.position[i('right_arm')]
            self.state[2] = msg.position[i('right_elbow')]
            self.state[3] = msg.position[i('right_gripper')]
            self.synced = True
        except ValueError:
            pass

    def _pub(self):
        m = Float64MultiArray()
        m.data = list(self.state)
        self.pub.publish(m)

    def _move_to_angles(self, s, a, e, g, label="Target"):
        """Publishes angles and waits for completion."""
        rospy.loginfo(f"  → Moving to {label}: [{s:.1f}, {a:.1f}, {e:.1f}, {g:.1f}]")
        m = Float64MultiArray()
        msg_data = [float(s), float(a), float(e), float(g)]
        m.data = msg_data
        self.pub.publish(m)
        
        # Wait for completion (within 1 degree)
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            diffs = [abs(self.state[i] - msg_data[i]) for i in range(4)]
            if max(diffs) < 1.0:
                rospy.loginfo(f"    ✓ {label} reached.")
                return True
            if (rospy.Time.now() - start_time).to_sec() > 20.0:
                rospy.logwarn(f"    ! {label} timeout.")
                return False
            rate.sleep()

    def execute_pick(self, tx, ty, tz):
        """Pick sequence: Home -> Above -> Pick -> Close. (Lifting happens after choice)"""
        res_target, _ = ik_search(tx, ty, tz)
        res_above,  _ = ik_search(tx, ty, tz + APPROACH_DZ)
        
        if res_target is None or res_above is None:
            rospy.logerr("Could not find IK for target or approach point.")
            return None

        s, a, e = res_target
        sa, aa, ea = res_above

        rospy.loginfo("=== Starting PICK phase ===")
        self._move_to_angles(90, 90, 90, 110, "Home (Open)")
        self._move_to_angles(sa, aa, ea, 110, "Above Target")
        self._move_to_angles(s, a, e, 110, "Pick Point")
        self._move_to_angles(s, a, e, 160, "Gripper Close")
        
        rospy.loginfo("=== Gripper closed at target. ===")
        # Return necessary data for the lift/move phase
        return {
            'target': (s, a, e),
            'above': (sa, aa, ea),
            'raw_xyz': (tx, ty, tz)
        }

    def execute_release(self, tx, ty, tz, current_xyz_above):
        """Vertical deposit sequence."""
        res_target, _ = ik_search(tx, ty, tz)
        res_above,  _ = ik_search(tx, ty, tz + APPROACH_DZ)
        
        if res_target is None or res_above is None:
            rospy.logerr("Release target unreachable.")
            return False

        s, a, e = res_target
        sa, aa, ea = res_above

        # Transition from wherever we were (likely current_xyz_above) to new location's above point
        rospy.loginfo("=== Transitioning to RELEASE point ===")
        self._move_to_angles(sa, aa, ea, 160, "Above New Target")
        self._move_to_angles(s, a, e, 160, "Release Point")
        self._move_to_angles(s, a, e, 110, "Release (Open)")
        self._move_to_angles(sa, aa, ea, 110, "Ascend Away")
        self._move_to_angles(90, 90, 90, 110, "Home Finish")
        return True


    def run(self):
        print("\n" + "="*62)
        print("  Companio Right Arm  —  Numerical IK")
        print("="*62)
        print(f"  L1={L1} cm  L2={L2} cm  OriginX={ORIGIN_X} cm")
        print(f"  Shoulder {SHOULDER_MIN}°–{SHOULDER_MAX}°  "
              f"Arm {ARM_MIN}°–{ARM_MAX}°  "
              f"Elbow {ELBOW_MIN}°–{ELBOW_MAX}°")
        print("  Coordinates in cm, global robot frame.")
        print("  'q' → quit\n" + "="*62 + "\n")

        while not rospy.is_shutdown():
            try:
                print("\n[PICK PHASE]")
                raw = input("Pick Target (X Y Z) or 'q' > ").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if raw.lower() in ('q', 'quit', 'exit'):
                break

            parts = raw.split()
            if len(parts) != 3:
                print("  ✗  Enter exactly 3 numbers: X Y Z")
                continue
            try:
                tx, ty, tz = [float(p) for p in parts]
            except ValueError:
                print("  ✗  Non-numeric value detected.")
                continue

            # Check target and approach safety
            blocked, zone = in_no_go(tx, ty, tz)
            if blocked:
                print(f"  ✗  PICK TARGET in '{zone}' — blocked.")
                continue
            blocked_a, zone_a = in_no_go(tx, ty, tz + APPROACH_DZ)
            if blocked_a:
                print(f"  ✗  APPROACH (+{APPROACH_DZ}cm) in '{zone_a}' — blocked.")
                continue

            # Confirm Prediction
            res_t, _ = ik_search(tx, ty, tz)
            if res_t is None:
                print("  ✗  Target point unreachable.")
                continue
            
            print(f"\n  Prediction: Shoulder={res_t[0]:.1f}, Arm={res_t[1]:.1f}, Elbow={res_t[2]:.1f}")
            if input("  Execute pick? [y/N] > ").strip().lower() != 'y':
                continue

            # EXECUTE PICK
            pick_data = self.execute_pick(tx, ty, tz)
            if not pick_data:
                continue

            # ASK USER IMMEDIATELY AFTER CLOSING
            print("\n  Gripper closed at target.")
            print("  Release Options: [H]ome, [N]ew Location, [S]tay here")
            choice = input("  Choice > ").strip().lower()

            if choice == 'h':
                # Lift then Home
                sa, aa, ea = pick_data['above']
                print("  Lifting and returning Home...")
                self._move_to_angles(sa, aa, ea, 160, "Lift Above")
                self._move_to_angles(90, 90, 90, 160, "Home (Carrying)")
                self._move_to_angles(90, 90, 90, 110, "Release (Open)")
            elif choice == 'n':
                # New Location logic
                while True:
                    raw_rel = input("  Release Target (X Y Z) > ").strip()
                    parts_rel = raw_rel.split()
                    if len(parts_rel) == 3:
                        try:
                            rx, ry, rz = [float(p) for p in parts_rel]
                            # Lift first
                            sa, aa, ea = pick_data['above']
                            self._move_to_angles(sa, aa, ea, 160, "Lift Above")
                            # Then release sequence
                            self.execute_release(rx, ry, rz, pick_data['above'])
                            break
                        except ValueError:
                            print("  ✗  Invalid numbers.")
                    else:
                        print("  ✗  Invalid format.")
            else:
                print("  Staying at current target (Closed).")

        print("Exiting.")


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == '__main__':
    NumericalIKNode().run()
