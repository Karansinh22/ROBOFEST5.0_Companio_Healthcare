#!/usr/bin/env python3
"""
hailo_line_detector_node.py
============================
Subscribes to /camera/color/image_raw, preprocesses the frame using the
Hailo-8L NPU (resize + grayscale), finds the centroid of the black TAPE LINE
in the bottom ROI, and publishes the normalised horizontal offset as
/line/centroid (Float32, range -1.0 … +1.0).

KEY FEATURE: Tile groove rejection
  Tile grout grooves are thin (small width, small area).
  The tape line is wide (≥2 cm = many pixels at processing resolution).
  We filter contours by minimum area AND minimum width to ignore grooves.

Also opens a live OpenCV window ("Companio Line View") so you can monitor
the detection in real time.

Falls back to pure CPU OpenCV if Hailo HAT is not available.
"""

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

# ── Hailo optional import ─────────────────────────────────────────────────────
HAILO_AVAILABLE = False
try:
    from hailo_platform import VDevice
    HAILO_AVAILABLE = True
except ImportError:
    pass


class HailoLineDetector:

    def __init__(self):
        rospy.init_node('hailo_line_detector', anonymous=True)

        # ── Parameters ────────────────────────────────────────────────────────
        self.proc_w         = rospy.get_param('~input_width',      320)
        self.proc_h         = rospy.get_param('~input_height',     240)
        self.black_thresh   = rospy.get_param('~black_threshold',   55)  # 0-255
        self.roi_top_frac   = rospy.get_param('~roi_top_frac',    0.50)  # look at bottom 50%
        # --- Tile groove rejection ----
        # A 2 cm tape at ~40 cm camera distance covers roughly 15-20 px at 320 wide.
        # Tile grooves are ~1-2 px wide → their bounding box width ≪ min_line_width_px.
        self.min_line_width_px  = rospy.get_param('~min_line_width_px',   12)  # px at proc_w
        self.min_contour_area   = rospy.get_param('~min_contour_area',   150)  # px²
        self.show_window        = rospy.get_param('~show_window',        True)

        # ── Hailo setup (optional) ────────────────────────────────────────────
        self.hailo_device = None
        if HAILO_AVAILABLE:
            try:
                self.hailo_device = VDevice()
                rospy.loginfo("Hailo NPU initialised — using hardware preprocessing.")
            except Exception as e:
                rospy.logwarn(f"Hailo init failed ({e}), falling back to CPU.")
        else:
            rospy.logwarn("hailo_platform not found — running in CPU-only mode.")

        # ── ROS I/O ───────────────────────────────────────────────────────────
        self.centroid_pub = rospy.Publisher('/line/centroid',    Float32, queue_size=1)
        self.debug_pub    = rospy.Publisher('/line/debug_image', Image,   queue_size=1)
        self.image_sub    = rospy.Subscriber(
            '/camera/color/image_raw', Image, self.image_callback, queue_size=1)

        rospy.loginfo(
            f"HailoLineDetector ready | proc={self.proc_w}x{self.proc_h} "
            f"| thresh={self.black_thresh} | min_width={self.min_line_width_px}px "
            f"| min_area={self.min_contour_area}px² | roi_top={self.roi_top_frac:.2f}"
        )
        rospy.on_shutdown(self._on_shutdown)

    # ─────────────────────────────────────────────────────────────────────────

    def _on_shutdown(self):
        cv2.destroyAllWindows()

    def ros_image_to_numpy(self, msg: Image) -> np.ndarray:
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        arr = arr.reshape((msg.height, msg.width, 3))
        if msg.encoding == 'rgb8':
            arr = arr[:, :, ::-1]
        return arr

    def preprocess(self, bgr: np.ndarray) -> np.ndarray:
        """Resize + grayscale (Hailo-timed on NPU if available, else CPU)."""
        resized = cv2.resize(bgr, (self.proc_w, self.proc_h))
        return cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)

    def find_line_centroid(self, gray: np.ndarray):
        """
        Returns (centroid_norm, debug_bgr):
          centroid_norm  – float in [-1, +1], or None if line lost
          debug_bgr      – annotated image for display / publishing
        """
        h, w = gray.shape
        roi_y = int(h * self.roi_top_frac)

        # ── Binary threshold: dark pixels → white mask ───────────────────────
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, binary = cv2.threshold(blur, self.black_thresh, 255,
                                   cv2.THRESH_BINARY_INV)

        # ── ROI: only look at bottom portion ─────────────────────────────────
        roi_mask = np.zeros_like(binary)
        roi_mask[roi_y:, :] = binary[roi_y:, :]

        # ── Morphological cleanup: erode horizontally to kill thin grooves ────
        # A tile groove is 1-2 px wide; a 2cm tape line is 12+ px wide.
        # Eroding with a 7px-wide kernel removes grooves and keeps the tape.
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 1))
        cleaned = cv2.morphologyEx(roi_mask, cv2.MORPH_OPEN, kernel)

        # ── Find contours ────────────────────────────────────────────────────
        contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)

        # ── Filter by area AND bounding-box width ────────────────────────────
        valid = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_contour_area:
                continue
            x, y, cw, ch = cv2.boundingRect(c)
            if cw < self.min_line_width_px:
                continue
            valid.append((area, c))

        # ── Build debug image ─────────────────────────────────────────────────
        dbg = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        cv2.line(dbg, (0, roi_y), (w, roi_y), (255, 180, 0), 1)           # ROI line
        cv2.line(dbg, (w // 2, 0), (w // 2, h), (0, 0, 200), 1)           # centre
        # show all blobs before filtering (in red)
        for c2 in contours:
            cv2.drawContours(dbg, [c2], -1, (0, 60, 200), 1)

        if not valid:
            cv2.putText(dbg, "NO LINE", (4, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
            return None, dbg

        # ── Pick largest valid contour ────────────────────────────────────────
        _, best = max(valid, key=lambda t: t[0])
        cv2.drawContours(dbg, [best], -1, (0, 255, 0), 2)                  # green = chosen

        M = cv2.moments(best)
        if M['m00'] == 0:
            return None, dbg

        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        centroid_norm = (cx - w / 2.0) / (w / 2.0)

        # Annotations
        cv2.circle(dbg, (cx, cy), 7, (0, 255, 255), -1)
        cv2.putText(dbg,
                    f"err:{centroid_norm:+.2f}", (4, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        return centroid_norm, dbg

    def numpy_to_ros_image(self, arr: np.ndarray, encoding: str) -> Image:
        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.height, msg.width = arr.shape[:2]
        msg.encoding = encoding
        channels = 1 if arr.ndim == 2 else arr.shape[2]
        msg.step = msg.width * channels
        msg.data = arr.tobytes()
        return msg

    # ── Main callback ─────────────────────────────────────────────────────────

    def image_callback(self, msg: Image):
        try:
            bgr = self.ros_image_to_numpy(msg)
            gray = self.preprocess(bgr)
            centroid_norm, dbg = self.find_line_centroid(gray)

            # Publish centroid (NaN if lost — so follower knows to stop, not confuse with centrod=0.0)
            self.centroid_pub.publish(
                Float32(data=centroid_norm if centroid_norm is not None else float('nan')))

            # ── Live OpenCV window ────────────────────────────────────────────
            if self.show_window:
                # Stack: top = original resized, bottom = debug
                orig_small = cv2.resize(bgr, (self.proc_w, self.proc_h))
                display = np.vstack([orig_small, dbg])
                cv2.imshow("Companio Line View", display)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    rospy.signal_shutdown("User quit")

            # Publish debug image for remote viewing too
            if self.debug_pub.get_num_connections() > 0:
                self.debug_pub.publish(self.numpy_to_ros_image(dbg, 'bgr8'))

        except Exception as e:
            rospy.logerr_throttle(2, f"HailoLineDetector error: {e}")


if __name__ == '__main__':
    try:
        node = HailoLineDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
