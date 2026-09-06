#!/usr/bin/env python3
"""
ai_destination_follower.py
==========================
Extends the line follower to navigate based on QR codes placed at junctions.
At junctions, it scans a QR code, parses the JSON payload, and looks up the route
for the user's target destination (e.g., "Room A" -> "turn_left", "turn_right", "straight").

Requires: pyzbar
"""

import time
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import json

# Attempt to import pyzbar
try:
    from pyzbar import pyzbar
    HAS_PYZBAR = True
except ImportError:
    HAS_PYZBAR = False

# State machine states
STATE_FOLLOW   = "FOLLOW"
STATE_DECIDE   = "DECIDE"     # junction found, looking for color marker
STATE_APPROACH = "APPROACH"   # corner detected, drive forward to align pivot
STATE_TURNING  = "TURNING"    # rotating through the corner
STATE_RECOVER  = "RECOVER"    # searching for line after turn

class AIDestinationFollower:

    def __init__(self):
        rospy.init_node('ai_destination_follower', anonymous=False)

        # Get destination from user (ROS param is cleaner for non-interactive nodes)
        self.target_destination = rospy.get_param('~destination', "").upper()
        if not self.target_destination:
            print("\n" + "="*50)
            print("🗺️  DESTINATION ROUTING ACTIVATED")
            self.target_destination = input("Enter destination (e.g., Room A, Room B): ").strip().upper()
            print("="*50 + "\n")
        
        if not self.target_destination:
            self.target_destination = "ROOM A" # Final fallback
        
        self.base_speed    = rospy.get_param('~base_speed',      0.315)
        self.turn_speed    = rospy.get_param('~turn_speed',      0.20)
        self.curve_turn_speed = rospy.get_param('~curve_turn_speed', 0.55)
        self.wheel_base    = rospy.get_param('~wheel_base',       0.95)
        self.speed_scale   = rospy.get_param('~speed_scale',       1.0)
        self.black_thresh  = rospy.get_param('~black_threshold',  55)
        self.roi_top_frac  = rospy.get_param('~roi_top_frac',   0.40)
        self.proc_w        = rospy.get_param('~proc_width',      320)
        self.proc_h        = rospy.get_param('~proc_height',     240)
        self.min_area      = rospy.get_param('~min_area',        150)
        self.min_width_px  = rospy.get_param('~min_width_px',     12)
        self.slight_thresh = rospy.get_param('~slight_thresh',  0.05)
        self.hard_thresh   = rospy.get_param('~hard_thresh',    0.35)
        self.lost_thresh   = rospy.get_param('~lost_thresh',      20)
        self.show_window   = rospy.get_param('~show_window',    True)

        # Injection Zone (Red)
        self.red_low1 = np.array([0, 100, 100]); self.red_high1 = np.array([10, 255, 255])
        self.red_low2 = np.array([160, 100, 100]); self.red_high2 = np.array([180, 255, 255])

        # Avoidance Zone (Red) - using existing red detection logic
        self.red_low1 = np.array([0, 100, 100]); self.red_high1 = np.array([10, 255, 255])
        self.red_low2 = np.array([160, 100, 100]); self.red_high2 = np.array([180, 255, 255])
        self.red_min_area = 500

        # Junction Params
        self.turn_aspect_min   = rospy.get_param('~turn_aspect_min',   4.0)
        self.turn_near_frac    = rospy.get_param('~turn_near_frac',   0.85)
        self.approach_duration_s = rospy.get_param('~approach_duration_s', 0.65)
        self.turn_duration_s     = rospy.get_param('~turn_duration_s',    2.5)
        self.recover_duration_s  = rospy.get_param('~recover_duration_s', 2.0)

        # State machine
        self._state          = STATE_FOLLOW
        self._turn_direction = None
        self._phase_start    = None
        self._lost_count     = 0
        self._corner_lost    = 0
        self._last_centroid  = 0.0
        self.startup_delay   = rospy.get_param('~startup_delay', 6.0)
        self._start_time     = None
        self._ready          = False
        
        # Junction decision vars
        self._decision_start_time = None
        self._detected_text_left = ""
        self._detected_text_right = ""

        # ROS I/O
        self.lwheel_pub = rospy.Publisher('/lwheel_vtarget', Float32, queue_size=1)
        self.rwheel_pub = rospy.Publisher('/rwheel_vtarget', Float32, queue_size=1)
        self.image_sub  = rospy.Subscriber(
            '/camera/color/image_raw', Image, self.image_callback, queue_size=1)

        self._last_img_time = rospy.Time.now()
        rospy.Timer(rospy.Duration(0.3), self._watchdog)
        rospy.on_shutdown(self._shutdown)

        if self.show_window:
            cv2.namedWindow("Destination Follower", cv2.WINDOW_NORMAL)

        rospy.loginfo("=" * 60)
        rospy.loginfo(f"AI Destination Follower | TARGET: {self.target_destination}")
        rospy.loginfo("Navigation Mode: QR Code JSON Routing")
        if not HAS_PYZBAR:
            rospy.logerr("pyzbar NOT FOUND! Please install it with: pip3 install pyzbar")
        rospy.loginfo("=" * 60)

    def ros_to_bgr(self, msg: Image) -> np.ndarray:
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        return arr[:, :, ::-1] if msg.encoding == 'rgb8' else arr.copy()

    def detect_line(self, bgr: np.ndarray):
        small = cv2.resize(bgr, (self.proc_w, self.proc_h))
        hsv   = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        gray  = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        blur  = cv2.GaussianBlur(gray, (5, 5), 0)

        _, binary = cv2.threshold(blur, self.black_thresh, 255, cv2.THRESH_BINARY_INV)

        roi_y = int(self.proc_h * self.roi_top_frac)
        mask  = np.zeros_like(binary)
        mask[roi_y:, :] = binary[roi_y:, :]

        # Red Detection
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

        # Black Line Detection
        kernel  = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 1))
        cleaned = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        result = {
            'centroid': None, 'is_corner': False,
            'corner_dir': None, 'corner_near': False,
            'red_detected': red_detected, 'red_centroid': red_centroid,
            'original': small, 'debug': None,
            'roi_l': None, 'roi_r': None # For visualization
        }

        best_c, best_area = None, 0
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_area: continue
            _, _, cw, _ = cv2.boundingRect(c)
            if cw < self.min_width_px: continue
            if area > best_area:
                best_area, best_c = area, c

        dbg = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        cv2.line(dbg, (0, roi_y), (self.proc_w, roi_y), (255, 180, 0), 1)
        cv2.line(dbg, (self.proc_w // 2, 0), (self.proc_w // 2, self.proc_h), (0, 0, 200), 1)

        if red_detected and best_red_c is not None:
            cv2.drawContours(dbg, [best_red_c], -1, (0, 0, 255), 2)

        if best_c is not None:
            bx, by, bw, bh = cv2.boundingRect(best_c)
            aspect = bw / max(bh, 1)
            M = cv2.moments(best_c)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                centroid_norm = (cx - self.proc_w / 2.0) / (self.proc_w / 2.0)
            else:
                centroid_norm = 0.0

            is_corner = aspect >= self.turn_aspect_min
            if is_corner:
                near_y = roi_y + self.turn_near_frac * (self.proc_h - roi_y)
                corner_near = by <= near_y
                result['is_corner']   = True
                result['corner_near'] = corner_near
                cv2.drawContours(dbg, [best_c], -1, (0, 165, 255), 2)
            else:
                result['centroid'] = centroid_norm
                cv2.drawContours(dbg, [best_c], -1, (0, 255, 0), 2)
        
        result['debug'] = dbg
        return result

    def scan_qr_code(self, bgr: np.ndarray):
        """ Scans the image for QR codes and returns parsed JSON data and debug overlay. """
        if not HAS_PYZBAR: return None, None
        
        debug_img = bgr.copy()
        decoded_objects = pyzbar.decode(bgr)
        
        if decoded_objects:
            obj = decoded_objects[0] # take the first found
            (x, y, w, h) = obj.rect
            cv2.rectangle(debug_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            qr_text = obj.data.decode('utf-8')
            try:
                payload = json.loads(qr_text)
                # The prompt shows Payload is wrapped inside {"payload": {"junction":...}} but also just the raw json.
                # If wrapped, extract. If not, use direct.
                if 'payload' in payload:
                    payload = payload['payload']
                return payload, debug_img
            except json.JSONDecodeError:
                rospy.logwarn(f"Failed to parse QR JSON: {qr_text}")
                return None, debug_img
                
        return None, debug_img

    def drive(self, v_w: tuple):
        v, w = v_w
        left_speed  =  (v - w * self.wheel_base / 2.0) * self.speed_scale
        right_speed = -(v + w * self.wheel_base / 2.0) * self.speed_scale
        self.lwheel_pub.publish(Float32(data=float(left_speed)))
        self.rwheel_pub.publish(Float32(data=float(right_speed)))

    def steer_proportional(self, c: float) -> tuple:
        v = self.base_speed
        if abs(c) < self.slight_thresh: return (v, 0.0)
        w = -c * self.curve_turn_speed
        return (v, w)

    def run_state_machine(self, det: dict, orig_bgr: np.ndarray) -> tuple:
        now = rospy.Time.now()
        s   = self._state

        if det['red_detected']:
            rospy.logwarn_throttle(1, "AVOIDING INFECTION ZONE")
            return (0.0, 0.0)

        if s == STATE_FOLLOW:
            if det['is_corner'] and det['corner_near']:
                self._state = STATE_DECIDE
                self._decision_start_time = now
                rospy.logwarn("[JUNCTION] Stopping to read signage...")
                return (0.0, 0.0)
            elif det['centroid'] is not None:
                self._last_centroid = det['centroid']
                return self.steer_proportional(det['centroid'])
            else:
                return (0.0, 0.0)

        elif s == STATE_DECIDE:
            # Scan for QR code
            elapsed = (now - self._decision_start_time).to_sec()
            
            payload, debug_img = self.scan_qr_code(orig_bgr)
            if debug_img is not None:
                # Optionally show bounding box on original view for debugging
                det['original'] = debug_img
            
            if payload and ('routes' in payload or 'r' in payload):
                junction_name = payload.get('junction', payload.get('j', 'Unknown'))
                rospy.loginfo_once(f"QR CODE DETECTED! Junction: {junction_name}")
                
                routes = payload.get('routes', payload.get('r', {}))
                # Case-insensitive matching for robust routing
                target = self.target_destination.lower()
                matched_route = None
                
                for room, route in routes.items():
                    # Check "room a" in "a" (False) or "a" in "room a" (True)
                    r_lower = room.lower()
                    if target == r_lower or target.endswith(r_lower) or r_lower.endswith(target):
                         matched_route = route.lower()
                         break
                    elif r_lower in target or target in r_lower:
                        matched_route = route.lower()
                        break
                
                if matched_route:
                    rospy.logwarn(f"ROUTE FOUND for '{self.target_destination}': {matched_route.upper()}")
                    
                    if matched_route in ["turn_left", "l"]:
                        self._turn_direction = 'j'
                        self._state = STATE_APPROACH
                    elif matched_route in ["turn_right", "r"]:
                        self._turn_direction = 'l'
                        self._state = STATE_APPROACH
                    elif matched_route in ["straight", "s"]:
                        # For straight, we don't need to approach and turn, we just resume following,
                        # but we need to push past the thick junction line first.
                        self._turn_direction = 'i' 
                        self._state = STATE_APPROACH
                    else:
                        rospy.logerr(f"Unknown route command: {matched_route}")
                        
                    self._phase_start = now
                else:
                    rospy.logwarn_throttle(2, f"Target '{self.target_destination}' NOT in QR routes: {list(routes.keys())}")
            else:
                if elapsed > 10.0:
                    rospy.logerr_throttle(2, "COULD NOT FIND QR CODE - STOPPING")
                    
            return (0.0, 0.0)

        elif s == STATE_APPROACH:
            if det['centroid'] is None or not det['is_corner']:
                self._corner_lost += 1
            else:
                self._corner_lost = 0
            if self._corner_lost >= 2:
                self._phase_start = now
                self._state       = STATE_TURNING
            return (self.base_speed, 0.0)

        elif s == STATE_TURNING:
            elapsed = (now - self._phase_start).to_sec()
            if elapsed >= self.turn_duration_s:
                self._phase_start = now
                self._state       = STATE_RECOVER
            w = self.turn_speed if self._turn_direction == 'j' else -self.turn_speed
            return (0.0, w)

        elif s == STATE_RECOVER:
            elapsed = (now - self._phase_start).to_sec()
            if det['centroid'] is not None:
                self._state = STATE_FOLLOW
            if elapsed >= self.recover_duration_s:
                self._state = STATE_FOLLOW
            return (self.base_speed, 0.0)

        return (0.0, 0.0)

    def image_callback(self, msg: Image):
        self._last_img_time = rospy.Time.now()
        if self._start_time is None: self._start_time = time.time()

        if not self._ready:
            if (time.time() - self._start_time) < self.startup_delay: return
            else: self._ready = True

        try:
            bgr = self.ros_to_bgr(msg)
            det = self.detect_line(bgr)
            cmd = self.run_state_machine(det, bgr)
            self.drive(cmd)

            if self.show_window:
                panel = np.vstack([det['original'], det['debug']])
                
                # Highlight scanning state
                if self._state == STATE_DECIDE:
                    cv2.putText(panel, "SCANNING QR CODE...", (panel.shape[1]//2 - 100, panel.shape[0]//2), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                cv2.putText(panel, f"TARGET: {self.target_destination}", (10, 20), 0, 0.6, (255, 255, 255), 2)
                cv2.putText(panel, f"STATE: {self._state}", (10, panel.shape[0]-10), 0, 0.5, (255, 255, 0), 1)
                cv2.imshow("Color Destination Follower", panel)
                cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"Error: {e}")

    def _watchdog(self, _):
        if (rospy.Time.now() - self._last_img_time).to_sec() > 0.5:
            self.drive((0,0))

    def _shutdown(self):
        self.drive((0,0))
        cv2.destroyAllWindows()

if __name__ == '__main__':
    # Interactive destination input if not provided via launch/params
    rospy.init_node('ai_destination_follower', anonymous=False)
    target = rospy.get_param('~destination', "")
    
    if target == "":
        print("\n" + "="*40)
        target = input("ENTER DESTINATION (e.g., ROOM A): ").upper().strip()
        print("="*40 + "\n")
        rospy.set_param('~destination', target)

    try:
        node = AIDestinationFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
