#!/usr/bin/env python3
import time
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

# State machine states
STATE_FOLLOW   = "FOLLOW"
STATE_APPROACH = "APPROACH"
STATE_TURNING  = "TURNING"
STATE_RECOVER  = "RECOVER"

class AIPathFollower:

    def __init__(self):
        rospy.init_node('ai_path_follower', anonymous=False)

        # Config exactly as specified by the user
        self.base_speed = 0.315
        self.turn_speed = 0.20
        # Decrease turn multiplier because steering from the top 50%
        # creates a huge "lever arm" that will cause oversteer
        self.curve_turn_speed = 0.6
        
        self.wheel_base = 0.95
        self.speed_scale = 1.0

        self.black_thresh = 55
        self.roi_top_frac = 0.40
        self.proc_w = 320
        self.proc_h = 240
        self.min_area = 150
        self.min_width_px = 2
        self.slight_thresh = 0.01
        self.lost_thresh = 20

        self.turn_aspect_min = 3.5
        self.turn_near_frac = 0.45
        self.approach_duration_s = 1.3  # Increased from 1.2
        self.turn_duration_s = 2.5
        self.recover_duration_s = 2.0
        
        # Hardware offset: If the camera is mounted slightly off-center left/right
        # Positive values shift the target center RIGHT, forcing the robot LEFT
        # Negative values shift the target center LEFT, forcing the robot RIGHT
        self.camera_offset_x = 0

        self._state = STATE_FOLLOW
        self._turn_direction = None
        self._phase_start = None
        self._corner_lost = 0

        self._lost_count = 0
        self._last_error = 0.0
        
        self.startup_delay = 8.0
        self._start_time = None
        self._ready = False

        self.lwheel_pub = rospy.Publisher('/lwheel_vtarget', Float32, queue_size=1)
        self.rwheel_pub = rospy.Publisher('/rwheel_vtarget', Float32, queue_size=1)
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback, queue_size=1)

        self._last_img_time = rospy.Time.now()
        rospy.Timer(rospy.Duration(0.5), self.watchdog)
        rospy.on_shutdown(self.shutdown)

        cv2.namedWindow("Companio Path Follower", cv2.WINDOW_NORMAL)
        cv2.setWindowProperty("Companio Path Follower", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        rospy.loginfo("=" * 60)
        rospy.loginfo("AI Path Follower (2 Lines)")
        rospy.loginfo("=" * 60)

    def ros_to_bgr(self, msg: Image) -> np.ndarray:
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        return arr[:, :, ::-1] if msg.encoding == 'rgb8' else arr.copy()

    def detect_lane(self, bgr: np.ndarray):
        small = cv2.resize(bgr, (self.proc_w, self.proc_h))
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        _, binary = cv2.threshold(blur, self.black_thresh, 255, cv2.THRESH_BINARY_INV)

        roi_y = int(self.proc_h * self.roi_top_frac)
        mask = np.zeros_like(binary)
        mask[roi_y:, :] = binary[roi_y:, :]

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 1))
        cleaned = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # --- Filter out dust, glare, and noise ---
        # Find all blobs in the cleaned threshold
        contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Create a perfectly blank slate
        clean_mask = np.zeros_like(cleaned)
        
        # Only draw the large, continuous track contours onto the blank slate
        for c in contours:
            if cv2.contourArea(c) >= self.min_area:
                cv2.drawContours(clean_mask, [c], -1, 255, -1)

        dbg = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        # ── 45%-55% Scan Chunk (For Turn Detection) ──
        # Looks further ahead to predict missing lines
        turn_y_start = int(self.proc_h * 0.45)
        turn_y_end   = int(self.proc_h * 0.55)
        cv2.rectangle(dbg, (0, turn_y_start), (self.proc_w, turn_y_end), (0, 165, 255), 1)
        
        # ── 65%-85% Scan Chunk (For Steering Stability) ──
        # Anchors steering close to the robot
        steer_y_start = int(self.proc_h * 0.65)
        steer_y_end   = int(self.proc_h * 0.85)
        cv2.rectangle(dbg, (0, steer_y_start), (self.proc_w, steer_y_end), (255, 180, 0), 1)
        
        # Calculate true center with physical mount offset
        center_x = (self.proc_w / 2.0) + self.camera_offset_x

        # Vertical target center line (blue)
        cv2.line(dbg, (int(center_x), 0), (int(center_x), self.proc_h), (255, 0, 0), 1)

        result = {
            'error': None,
            'is_corner': False,
            'corner_dir': None,
            'corner_near': False,
            'original': small,
            'debug': dbg,
            'cxs': []
        }

        # --- Process Steering Chunk ---
        # Look specifically at the perfectly clean, noise-free mask we just generated
        chunk = clean_mask[steer_y_start:steer_y_end, :]
        col_sums = np.sum(chunk, axis=0)
        # Accept any columns that have at least 1 white pixel
        white_pixels_steer = np.where(col_sums > 0)[0]
        
        clusters_steer = []
        if len(white_pixels_steer) > 0:
            current_cluster = [white_pixels_steer[0]]
            for x in white_pixels_steer[1:]:
                if x - current_cluster[-1] < 10:
                    current_cluster.append(x)
                else:
                    if len(current_cluster) >= self.min_width_px:
                        clusters_steer.append(current_cluster)
                    current_cluster = [x]
            if len(current_cluster) >= self.min_width_px:
                clusters_steer.append(current_cluster)

        steer_points = []
        chunk_mid_y = int((steer_y_start + steer_y_end) / 2)
        for cluster in clusters_steer:
            steer_points.append((int(np.mean(cluster)), chunk_mid_y))
        steer_points.sort(key=lambda p: p[0])
        if len(steer_points) > 2:
            steer_points = [steer_points[0], steer_points[-1]]
        
        if len(steer_points) == 2:
            left_pt = steer_points[0]
            right_pt = steer_points[1]
            mid_x = (left_pt[0] + right_pt[0]) / 2.0
            
            cv2.circle(dbg, left_pt, 6, (0, 0, 255), -1)
            cv2.circle(dbg, right_pt, 6, (0, 0, 255), -1)
            cv2.circle(dbg, (int(mid_x), chunk_mid_y), 8, (0, 255, 255), -1)
            cv2.line(dbg, left_pt, right_pt, (255, 0, 255), 2)
            
            error = (mid_x - center_x) / center_x
            result['error'] = error
            cv2.putText(dbg, f"PATH err:{error:+.2f}", (4, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 2)
        elif len(steer_points) == 1:
            # 1 line on steer pass. Ignore it to maintain straight trajectory.
            cv2.circle(dbg, steer_points[0], 6, (100, 100, 100), -1)
            cv2.putText(dbg, "1 LINE IGNORED - MAINTAINING", (4, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (100, 100, 100), 1)
            result['error'] = 0.0
        else:
            cv2.putText(dbg, "NO LINES (STEER)", (4, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
            return result

        # --- Process Turn Chunk (15%-25%) ---
        # Only check for turns if we are currently generally straight (error is low)
        if result['error'] is not None and abs(result['error']) < 0.3:
            # Look specifically at the perfectly clean, noise-free mask we just generated
            chunk_turn = clean_mask[turn_y_start:turn_y_end, :]
            col_sums_turn = np.sum(chunk_turn, axis=0)
            white_pixels_turn = np.where(col_sums_turn > 0)[0]
            
            clusters_turn = []
            if len(white_pixels_turn) > 0:
                current_cluster = [white_pixels_turn[0]]
                for x in white_pixels_turn[1:]:
                    if x - current_cluster[-1] < 10:
                        current_cluster.append(x)
                    else:
                        if len(current_cluster) >= self.min_width_px:
                            clusters_turn.append(current_cluster)
                        current_cluster = [x]
                if len(current_cluster) >= self.min_width_px:
                    clusters_turn.append(current_cluster)

            turn_points = []
            turn_mid_y = int((turn_y_start + turn_y_end) / 2)
            for cluster in clusters_turn:
                turn_points.append((int(np.mean(cluster)), turn_mid_y))
            turn_points.sort(key=lambda p: p[0])
            if len(turn_points) > 2:
                turn_points = [turn_points[0], turn_points[-1]]

            # We mark dots on the 30% line orange
            for p in turn_points:
                cv2.circle(dbg, p, 4, (0, 165, 255), -1)

            # Turn Trigger: Exactly 1 line visible at 30% height
            if len(turn_points) == 1:
                single_x = turn_points[0][0]
                if single_x < self.proc_w / 2.0:
                    result['corner_dir'] = 'l'
                    cv2.putText(dbg, "MISSING RIGHT -> TURN RIGHT", (4, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 165, 255), 2)
                else:
                    result['corner_dir'] = 'j'
                    cv2.putText(dbg, "MISSING LEFT -> TURN LEFT", (4, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 165, 255), 2)
                
                result['is_corner'] = True
                result['corner_near'] = True 
                result['error'] = 0.0 # Force straight while approaching

        
        return result

    def steer_proportional(self, error: float) -> tuple:
        v = self.base_speed
        if abs(error) < self.slight_thresh:
            return (v, 0.0)
            
        w = -error * self.curve_turn_speed
        return (v, w)

    def drive(self, v_w: tuple):
        v, w = v_w
        left_speed = (v - w * self.wheel_base / 2.0) * self.speed_scale
        right_speed = -(v + w * self.wheel_base / 2.0) * self.speed_scale
        self.lwheel_pub.publish(Float32(data=float(left_speed)))
        self.rwheel_pub.publish(Float32(data=float(right_speed)))

    def run_state_machine(self, det: dict) -> tuple:
        now = rospy.Time.now()
        s = self._state

        if s == STATE_FOLLOW:
            if det['is_corner'] and det['corner_near']:
                self._turn_direction = det['corner_dir']
                self._phase_start    = now
                self._state          = STATE_APPROACH
                self._corner_lost    = 0
                rospy.logwarn(f"[CORNER] {'RIGHT' if det['corner_dir'] == 'l' else 'LEFT'}! Approaching...")
                return (self.base_speed, 0.0)

            elif det['error'] is not None:
                self._last_error = det['error']
                self._lost_count = 0
                return self.steer_proportional(det['error'])
            else:
                self._lost_count += 1
                if self._lost_count >= self.lost_thresh:
                    rospy.logwarn_throttle(1, "Lines LOST too long — stopping.")
                    return (0.0, 0.0)
                recovery_cmd = self.steer_proportional(self._last_error)
                return recovery_cmd

        elif s == STATE_APPROACH:
            if det['error'] is None or not det['is_corner']:
                self._corner_lost += 1
            else:
                self._corner_lost = 0

            elapsed = (now - self._phase_start).to_sec()

            if self._corner_lost >= 2 or elapsed > self.approach_duration_s:
                self._phase_start = now
                self._state       = STATE_TURNING
                rospy.logwarn(f"[TURNING] {'RIGHT' if self._turn_direction == 'l' else 'LEFT'} for {self.turn_duration_s}s")
            return (self.base_speed, 0.0)

        elif s == STATE_TURNING:
            elapsed = (now - self._phase_start).to_sec()
            remaining = self.turn_duration_s - elapsed
            rospy.loginfo_throttle(0.5, f"[TURNING] {remaining:.1f}s remaining")
            if elapsed >= self.turn_duration_s:
                self._phase_start = now
                self._state       = STATE_RECOVER
                rospy.logwarn("[RECOVER] Turn done — searching for path...")
            
            w = self.turn_speed if self._turn_direction == 'j' else -self.turn_speed
            return (0.0, w)

        elif s == STATE_RECOVER:
            elapsed = (now - self._phase_start).to_sec()
            if det['error'] is not None and not det['is_corner']:
                rospy.loginfo("[FOLLOW] Path re-acquired!")
                self._state      = STATE_FOLLOW
                self._lost_count = 0
                return self.steer_proportional(det['error'])
            if elapsed >= self.recover_duration_s:
                rospy.logwarn("[FOLLOW] Recover timeout — resuming normal follow.")
                self._state = STATE_FOLLOW
                return (0.0, 0.0)
            return (self.base_speed, 0.0)

        return (0.0, 0.0)

    def image_callback(self, msg: Image):
        self._last_img_time = rospy.Time.now()

        if self._start_time is None:
            self._start_time = time.time()

        if not self._ready:
            elapsed = time.time() - self._start_time
            remaining = max(0, self.startup_delay - elapsed)
            if remaining > 0:
                bgr = self.ros_to_bgr(msg)
                small = cv2.resize(bgr, (self.proc_w, self.proc_h))
                blank = np.zeros_like(small)
                panel = np.vstack([small, blank])
                cv2.putText(panel, f"Starting in {remaining:.0f}s ...",
                            (self.proc_w // 2 - 75, self.proc_h // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
                cv2.putText(panel, "Camera warming up",
                            (self.proc_w // 2 - 70, self.proc_h // 2 + 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
                cv2.imshow("Companio Path Follower", panel)
                cv2.waitKey(1)
                return
            else:
                self._ready = True
                rospy.loginfo("[READY] Camera warm-up done — lane following ACTIVE!")

        try:
            bgr = self.ros_to_bgr(msg)
            det = self.detect_lane(bgr)

            cmd = self.run_state_machine(det)
            self.drive(cmd)

            orig = det['original']
            dbg = det['debug']
            panel = np.vstack([orig, dbg])
            
            cmd_str = f"v:{cmd[0]:.2f} w:{cmd[1]:.2f}"
            state_label = f"STATE:{self._state} Cmd:[{cmd_str}] "
            cv2.putText(panel, state_label, (4, panel.shape[0] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 255, 0), 1)
            
            cv2.imshow("Companio Path Follower", panel)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown("User quit")

        except Exception as e:
            rospy.logerr_throttle(2, f"Error: {e}")

    def watchdog(self, _):
        if (rospy.Time.now() - self._last_img_time).to_sec() > 0.5:
            self.lwheel_pub.publish(Float32(data=0.0))
            self.rwheel_pub.publish(Float32(data=0.0))

    def shutdown(self):
        self.lwheel_pub.publish(Float32(data=0.0))
        self.rwheel_pub.publish(Float32(data=0.0))
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        node = AIPathFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
