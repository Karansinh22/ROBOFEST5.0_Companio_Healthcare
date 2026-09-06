#!/usr/bin/env python3
"""
ai_line_follower_direct.py
===========================
All-in-one: camera → line detection + 90° corner detection → teleop key logic → motors.
Publishes directly to /lwheel_vtarget and /rwheel_vtarget.

Robot geometry (calibrated from physical measurements):
  Track width (wheel center-to-center) : 16 cm = 0.16 m
  Camera height from ground            : 21 cm
  Camera forward offset from axle      : +7 cm
  Optimal line detection range         : 15–60 cm ahead
  Max speed                            : 0.4–0.6 m/s

90° TURN: Time-based (not frame-based).
  - Approach phase  : drive forward for approach_duration_s (~0.15 s)
    to position wheel axle at the corner (camera overhang = 7 cm)
  - Turning phase   : rotate in place for turn_duration_s (3.0 s)
  - Recover phase   : creep forward for recover_duration_s (2.0 s)
    to find line again

Motor physical convention (Companio, VERIFIED):
  lwheel_vtarget > 0  → left  wheel CCW → robot moves FORWARD on left side
  rwheel_vtarget < 0  → right wheel CW  → robot moves FORWARD on right side
  (right motor is mounted inverted)

Teleop key equivalents:
  'i' forward        l=+B, r=-B
  'j' left rotate    l=-B, r=-B
  'l' right rotate   l=+B, r=+B
  'u' forward-left   l=+B/2, r=-B
  'o' forward-right  l=+B,   r=-B/2
  'k' stop           l=0,    r=0
"""

import time
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

TURN_DIVIDER = 2.0   # inner wheel speed = base_speed / TURN_DIVIDER during arcs

# State machine states
STATE_FOLLOW   = "FOLLOW"     # normal line following
STATE_APPROACH = "APPROACH"   # corner detected, drive forward to align pivot
STATE_TURNING  = "TURNING"    # rotating through the corner
STATE_RECOVER  = "RECOVER"    # searching for line after turn


class AILineFollowerDirect:

    def __init__(self):
        rospy.init_node('ai_line_follower_direct', anonymous=False)

        self.base_speed    = rospy.get_param('~base_speed',      0.315)
        self.turn_speed    = rospy.get_param('~turn_speed',      0.20)  # rotation speed for 90° corners
        self.curve_turn_speed = rospy.get_param('~curve_turn_speed', 0.55) # max steering speed for curves
        self.wheel_base    = rospy.get_param('~wheel_base',       0.95) # matches twist_to_motors.py
        self.wheel_base    = rospy.get_param('~wheel_base',       0.95) # matches twist_to_motors.py
        self.speed_scale   = rospy.get_param('~speed_scale',       1.0) # teleop sends raw unscaled v/w math
        self.black_thresh  = rospy.get_param('~black_threshold',  55)
        self.roi_top_frac  = rospy.get_param('~roi_top_frac',   0.40)  # look further ahead for curves
        self.proc_w        = rospy.get_param('~proc_width',      320)
        self.proc_h        = rospy.get_param('~proc_height',     240)
        self.min_area      = rospy.get_param('~min_area',        150)
        self.min_width_px  = rospy.get_param('~min_width_px',     12)
        self.slight_thresh = rospy.get_param('~slight_thresh',  0.05)  # reduced deadband -> react sooner
        self.hard_thresh   = rospy.get_param('~hard_thresh',    0.35)  # rotate zone
        self.show_window   = rospy.get_param('~show_window',    True)
        
        # ── Red (Infection Zone) detection params ──────────────────────────────
        # Red usually spans two ranges in HSV (0-10 and 160-180)
        self.red_low1 = np.array([0, 100, 100])
        self.red_high1 = np.array([10, 255, 255])
        self.red_low2 = np.array([160, 100, 100])
        self.red_high2 = np.array([180, 255, 255])
        self.red_min_area = 500  # Larger area threshold for red zones to avoid noise

        # ── 90° turn params ───────────────────────────────────────────────────
        # A straight-ahead line has a tall bounding box.
        # A 90° corner makes the line horizontal → wide bounding box.
        self.turn_aspect_min   = rospy.get_param('~turn_aspect_min',   4.0)
        # How close (fraction of proc_h from bottom) before we start the approach
        self.turn_near_frac    = rospy.get_param('~turn_near_frac',   0.85)
        # Approach: seconds to drive forward before rotating
        self.approach_duration_s = rospy.get_param('~approach_duration_s', 0.65)
        # Turn: 2.5 s for 90° rotation (user-specified)
        self.turn_duration_s     = rospy.get_param('~turn_duration_s',    2.5)
        # Recover: creep forward for 2.0 s to re-find the line
        self.recover_duration_s  = rospy.get_param('~recover_duration_s', 2.0)

        # ── State machine ─────────────────────────────────────────────────────
        self._state          = STATE_FOLLOW
        self._turn_direction = None
        self._phase_start    = None   # rospy.Time when current timed phase began
        self._lost_count     = 0
        self._corner_lost    = 0      # frames the corner has been absent during approach
        self._last_centroid  = 0.0
        # Startup delay: camera is black for ~5 s after launch
        self.startup_delay   = rospy.get_param('~startup_delay', 6.0)
        self._start_time     = None
        self._ready          = False

        # ── ROS I/O ─────────────────────────────────────────────────────
        self.lwheel_pub = rospy.Publisher('/lwheel_vtarget', Float32, queue_size=1)
        self.rwheel_pub = rospy.Publisher('/rwheel_vtarget', Float32, queue_size=1)
        self.image_sub  = rospy.Subscriber(
            '/camera/color/image_raw', Image, self.image_callback, queue_size=1)

        self._last_img_time = rospy.Time.now()
        rospy.Timer(rospy.Duration(0.3), self._watchdog)
        rospy.on_shutdown(self._shutdown)

        if self.show_window:
            cv2.namedWindow("Companio Line Follower", cv2.WINDOW_NORMAL)
            cv2.setWindowProperty("Companio Line Follower", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        rospy.loginfo("=" * 60)
        rospy.loginfo("AI Line Follower Direct  (with 90° corner detection)")
        rospy.loginfo(f"  base_speed={self.base_speed} | aspect_min={self.turn_aspect_min}")
        rospy.loginfo(f"  turn_duration={self.turn_duration_s}s | approach={self.approach_duration_s}s | recover={self.recover_duration_s}s")
        rospy.loginfo("=" * 60)

    # ── Image helpers ─────────────────────────────────────────────────────────

    def ros_to_bgr(self, msg: Image) -> np.ndarray:
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        return arr[:, :, ::-1] if msg.encoding == 'rgb8' else arr.copy()

    def detect_line(self, bgr: np.ndarray):
        """
        Returns dict with keys:
          'centroid'    : float -1..+1  or  None
          'is_corner'   : bool
          'corner_dir'  : 'j' (left) | 'l' (right) | None
          'corner_near' : bool (corner is close enough to trigger approach)
          'red_detected': bool
          'red_centroid': float -1..+1 or None
          'original'    : resized frame
          'debug'       : annotated debug frame
        """
        small = cv2.resize(bgr, (self.proc_w, self.proc_h))
        hsv   = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        gray  = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        blur  = cv2.GaussianBlur(gray, (5, 5), 0)

        _, binary = cv2.threshold(blur, self.black_thresh, 255, cv2.THRESH_BINARY_INV)

        roi_y = int(self.proc_h * self.roi_top_frac)
        mask  = np.zeros_like(binary)
        mask[roi_y:, :] = binary[roi_y:, :]

        # ── Red Detection ─────────────────────────────────────────────────────
        red_mask1 = cv2.inRange(hsv, self.red_low1, self.red_high1)
        red_mask2 = cv2.inRange(hsv, self.red_low2, self.red_high2)
        red_mask  = cv2.bitwise_or(red_mask1, red_mask2)
        red_mask_roi = np.zeros_like(red_mask)
        red_mask_roi[roi_y:, :] = red_mask[roi_y:, :]

        red_contours, _ = cv2.findContours(red_mask_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        red_detected = False
        red_centroid = None
        
        max_red_area = 0
        best_red_c = None
        for rc in red_contours:
            area = cv2.contourArea(rc)
            if area > self.red_min_area and area > max_red_area:
                max_red_area = area
                best_red_c = rc
                red_detected = True

        if red_detected and best_red_c is not None:
            RM = cv2.moments(best_red_c)
            if RM['m00'] > 0:
                rcx = int(RM['m10'] / RM['m00'])
                red_centroid = (rcx - self.proc_w / 2.0) / (self.proc_w / 2.0)

        # Kill tile grooves (thin horizontal lines) for black line detection
        kernel  = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 1))
        cleaned = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        result = {
            'centroid': None, 'is_corner': False,
            'corner_dir': None, 'corner_near': False,
            'red_detected': red_detected, 'red_centroid': red_centroid,
            'original': small, 'debug': None
        }

        best_c, best_area = None, 0
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_area:
                continue
            _, _, cw, _ = cv2.boundingRect(c)
            if cw < self.min_width_px:
                continue
            if area > best_area:
                best_area, best_c = area, c

        # ── Build debug frame ─────────────────────────────────────────────────
        dbg = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        cv2.line(dbg, (0, roi_y), (self.proc_w, roi_y), (255, 180, 0), 1)
        cv2.line(dbg, (self.proc_w // 2, 0), (self.proc_w // 2, self.proc_h), (0, 0, 200), 1)

        if red_detected and best_red_c is not None:
            cv2.drawContours(dbg, [best_red_c], -1, (0, 0, 255), 2)
            cv2.putText(dbg, "!!! INFECTION ZONE !!!", (self.proc_w // 2 - 80, roi_y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        if best_c is None:
            cv2.putText(dbg, "NO LINE", (4, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
            result['debug'] = dbg
            return result

        # ── Analyse contour ───────────────────────────────────────────────────
        bx, by, bw, bh = cv2.boundingRect(best_c)
        aspect = bw / max(bh, 1)

        M = cv2.moments(best_c)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            centroid_norm = (cx - self.proc_w / 2.0) / (self.proc_w / 2.0)
        else:
            centroid_norm = 0.0

        # ── Corner detection ──────────────────────────────────────────────────
        is_corner = aspect >= self.turn_aspect_min

        if is_corner:
            # Direction: does the line extend to the right or left of centre?
            # Compare left half vs right half mass
            left_mass  = bx + bw / 4       # quarter of line from left edge
            right_mass = bx + 3 * bw / 4   # quarter of line from right edge
            # Simpler: centroid sign tells us which way the tap goes
            # But for a symmetric T-junction this might be 0 — use bounding box skew
            line_center_x = bx + bw / 2.0
            bias = (line_center_x - self.proc_w / 2.0) / (self.proc_w / 2.0)

            if bias >= 0:
                corner_dir = 'l'    # line extends right → turn right
            else:
                corner_dir = 'j'    # line extends left  → turn left

            # Distance proxy: top of bounding box in ROI. Higher = farther.
            # near_threshold_y = roi_y + turn_near_frac * (proc_h - roi_y)
            near_y      = roi_y + self.turn_near_frac * (self.proc_h - roi_y)
            corner_near = by <= near_y   # blob top is within the near zone

            result['is_corner']   = True
            result['corner_dir']  = corner_dir
            result['corner_near'] = corner_near

            cv2.drawContours(dbg, [best_c], -1, (0, 165, 255), 2)   # orange = corner
            cv2.rectangle(dbg, (bx, by), (bx + bw, by + bh), (0, 165, 255), 1)
            cv2.putText(dbg,
                        f"CORNER {'RIGHT' if corner_dir == 'l' else 'LEFT'}  "
                        f"asp:{aspect:.1f}  near:{corner_near}",
                        (4, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 165, 255), 1)
        else:
            result['centroid'] = centroid_norm
            cy_dot = int((roi_y + self.proc_h) / 2)
            cv2.drawContours(dbg, [best_c], -1, (0, 255, 0), 2)
            cv2.circle(dbg, (int(centroid_norm * self.proc_w / 2 + self.proc_w / 2), cy_dot),
                       7, (0, 255, 255), -1)
            cv2.putText(dbg, f"LINE err:{centroid_norm:+.2f}  asp:{aspect:.1f}",
                        (4, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)

        result['debug'] = dbg
        return result

    # ── Kinematic motor interface ─────────────────────────────────────────────

    def drive(self, v_w: tuple):
        v, w = v_w
        # Identical kinematic math to motor_control_node.py
        # Right motor is physically inverted -> negated.
        # angular.z > 0 = turn LEFT
        left_speed  =  (v - w * self.wheel_base / 2.0) * self.speed_scale
        right_speed = -(v + w * self.wheel_base / 2.0) * self.speed_scale

        self.lwheel_pub.publish(Float32(data=float(left_speed)))
        self.rwheel_pub.publish(Float32(data=float(right_speed)))

    def steer_proportional(self, c: float) -> tuple:
        """ Returns (v, w) based on centroid offset """
        v = self.base_speed
        
        # Deadband for slight variations — go perfectly straight
        if abs(c) < self.slight_thresh:
            return (v, 0.0)
            
        # c > 0 means line is to the right. We want to turn right (w < 0).
        # We scale the angular velocity proportionally to the error using curve_turn_speed.
        w = -c * self.curve_turn_speed
        
        return (v, w)

    # ── State machine ─────────────────────────────────────────────────────────

    def run_state_machine(self, det: dict) -> tuple:
        """
        Returns the (v, w) velocities to command this frame.
        All timed phases use wall-clock time (rospy.Time) so behaviour is
        independent of camera frame rate.
        """
        now = rospy.Time.now()
        s   = self._state

        if det['red_detected']:
            rospy.logwarn_throttle(1, "AVOIDING INFECTION ZONE (Red Line Detected)")
            
            # If we see a black line as well, try to steer AWAY from the red zone
            if det['centroid'] is not None:
                # If red is on the right (centroid > 0), and black is on the left
                # or vice versa, we can try to follow the black line while favoring the side away from red.
                # For simplicity, if red is detected, we check if we can offset our steering.
                
                # If red and black are close, it's safer to stop.
                if abs(det['centroid'] - det['red_centroid']) < 0.4:
                    rospy.logwarn_throttle(1, "Red line blocking path - STOPPING")
                    return (0.0, 0.0)
                
                # Otherwise, follow the black line but with caution (slower)
                v, w = self.steer_proportional(det['centroid'])
                return (v * 0.5, w)
            else:
                # No clear alternative path, stop.
                return (0.0, 0.0)

        if s == STATE_FOLLOW:
            if det['is_corner'] and det['corner_near']:
                self._turn_direction = det['corner_dir']
                self._phase_start    = now
                self._state          = STATE_APPROACH
                self._corner_lost    = 0
                rospy.logwarn(
                    f"[CORNER] {'RIGHT' if det['corner_dir'] == 'l' else 'LEFT'}! "
                    f"Approaching until turn is lost under chassis...")
                return (self.base_speed, 0.0)

            elif det['centroid'] is not None:
                self._last_centroid = det['centroid']
                self._lost_count    = 0
                return self.steer_proportional(det['centroid'])
            else:
                self._lost_count += 1
                if self._lost_count >= self.lost_thresh:
                    rospy.logwarn_throttle(1, "Line LOST too long — stopping.")
                    return (0.0, 0.0)
                recovery_cmd = self.steer_proportional(self._last_centroid)
                rospy.loginfo_throttle(0.4,
                    f"[LOST {self._lost_count}/{self.lost_thresh}] "
                    f"last_centroid:{self._last_centroid:+.2f} → '{recovery_cmd}'")
                return recovery_cmd

        elif s == STATE_APPROACH:
            # Drive forward until we drive OVER the corner and it is lost from view.
            # Require it to be lost for 2 consecutive frames to avoid noise.
            if det['centroid'] is None or not det['is_corner']:
                self._corner_lost += 1
            else:
                self._corner_lost = 0

            # Failsafe timeout just in case the line never disappears (e.g. 5 seconds)
            elapsed = (now - self._phase_start).to_sec()

            if self._corner_lost >= 2 or elapsed > 5.0:
                self._phase_start = now
                self._state       = STATE_TURNING
                rospy.logwarn(
                    f"[TURNING] {'RIGHT' if self._turn_direction == 'l' else 'LEFT'} "
                    f"for {self.turn_duration_s}s")
            return (self.base_speed, 0.0)

        elif s == STATE_TURNING:
            # Rotate in place for turn_duration_s (user-specified)
            elapsed = (now - self._phase_start).to_sec()
            remaining = self.turn_duration_s - elapsed
            rospy.loginfo_throttle(0.5, f"[TURNING] {remaining:.1f}s remaining")
            if elapsed >= self.turn_duration_s:
                self._phase_start = now
                self._state       = STATE_RECOVER
                rospy.logwarn("[RECOVER] Turn done — searching for line...")
            
            w = self.turn_speed if self._turn_direction == 'j' else -self.turn_speed
            return (0.0, w)

        elif s == STATE_RECOVER:
            # Creep forward for recover_duration_s, then return to FOLLOW regardless
            elapsed = (now - self._phase_start).to_sec()
            if det['centroid'] is not None and not det['is_corner']:
                rospy.loginfo("[FOLLOW] Line re-acquired!")
                self._state      = STATE_FOLLOW
                self._lost_count = 0
                return self.steer_proportional(det['centroid'])
            if elapsed >= self.recover_duration_s:
                rospy.logwarn("[FOLLOW] Recover timeout — resuming normal follow.")
                self._state = STATE_FOLLOW
                return (0.0, 0.0)
            return (self.base_speed, 0.0)

        return (0.0, 0.0)

    # ── Main callback ─────────────────────────────────────────────────────────

    def image_callback(self, msg: Image):
        self._last_img_time = rospy.Time.now()

        # ── Startup delay: use wall-clock time so it doesn't freeze in sim/ros time
        if self._start_time is None:
            self._start_time = time.time()   # use python time.time()

        if not self._ready:
            elapsed   = time.time() - self._start_time
            remaining = max(0, self.startup_delay - elapsed)
            if remaining > 0:
                if self.show_window:
                    bgr   = self.ros_to_bgr(msg)
                    small = cv2.resize(bgr, (self.proc_w, self.proc_h))
                    # Create blank bottom panel so window size matches main loop
                    blank = np.zeros_like(small)
                    panel = np.vstack([small, blank])
                    cv2.putText(panel,
                                f"Starting in {remaining:.0f}s ...",
                                (self.proc_w // 2 - 75, self.proc_h // 2),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
                    cv2.putText(panel, "Camera warming up",
                                (self.proc_w // 2 - 70, self.proc_h // 2 + 22),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
                    cv2.imshow("Companio Line Follower", panel)
                    cv2.waitKey(1)
                return   # motors stay off during warmup
            else:
                self._ready = True
                rospy.loginfo("[READY] Camera warm-up done — line following ACTIVE!")

        try:
            bgr = self.ros_to_bgr(msg)
            det = self.detect_line(bgr)
            cmd = self.run_state_machine(det)
            self.drive(cmd)

            if self.show_window:
                orig  = det['original']
                dbg   = det['debug']
                panel = np.vstack([orig, dbg])
                
                # Format string
                cmd_str = f"v:{cmd[0]:.2f} w:{cmd[1]:.2f}"

                # State banner
                state_label = (
                    f"STATE:{self._state}  Cmd:[{cmd_str}] "
                )
                cv2.putText(panel, state_label,
                            (4, panel.shape[0] - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 255, 0), 1)
                if det['red_detected']:
                    cv2.putText(panel, "AVOIDING INFECTION ZONE",
                                (4, panel.shape[0] - 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                cv2.imshow("Companio Line Follower", panel)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    rospy.signal_shutdown("User quit")

        except Exception as e:
            rospy.logerr_throttle(2, f"Error: {e}")

    # ── Safety ────────────────────────────────────────────────────────────────

    def _watchdog(self, _):
        if (rospy.Time.now() - self._last_img_time).to_sec() > 0.5:
            self.lwheel_pub.publish(Float32(data=0.0))
            self.rwheel_pub.publish(Float32(data=0.0))

    def _shutdown(self):
        self.lwheel_pub.publish(Float32(data=0.0))
        self.rwheel_pub.publish(Float32(data=0.0))
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        node = AILineFollowerDirect()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
