#!/usr/bin/env python3
import cv2
import subprocess
import threading
import time
import json
import os
import signal
import sys
import numpy as np
from flask import Flask, Response, request, jsonify

# ROS Imports
try:
    import rospy
    from std_msgs.msg import Bool
except ImportError:
    print("ERROR: ROS packages not found. Running in standalone mode.")
    rospy = None

app = Flask(__name__)

# File Paths
# Use absolute paths relative to the script location if needed, 
# but for now assume current working directory (Docker run context)
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
AOI_CONFIG_FILE = os.path.join(SCRIPT_DIR, 'aoi_config.json')
CONFIG_FILE = os.path.join(SCRIPT_DIR, 'config.json')

def load_aoi():
    """Load AOIs for detection."""
    if os.path.exists(AOI_CONFIG_FILE):
        try:
            with open(AOI_CONFIG_FILE, 'r') as f:
                return json.load(f)
        except Exception as e:
            print(f"Error loading AOI: {e}")
    return {"left": [0, 0, 320, 240], "right": [0, 0, 320, 240]}

def load_config():
    """Load hardware config."""
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, 'r') as f:
                return json.load(f)
        except Exception as e:
            print(f"Error loading config: {e}")
    return {"left_camera": 0, "right_camera": 1}

config = load_config()
aois = load_aoi()

class GripperVisionBroadcaster:
    def __init__(self, side):
        self.side = side
        self.current_frame = None
        self.running = False
        self.condition = threading.Condition()
        self.confidence = 0.0
        self.detected = False
        
        # ROS Publisher
        if rospy:
            topic = f"/gripper_vision/{side}/target_detected"
            self.pub = rospy.Publisher(topic, Bool, queue_size=1)
        else:
            self.pub = None

    def start(self):
        self.running = True
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while self.running:
            # Refresh local copies of config
            global config, aois
            config = load_config()
            aois = load_aoi()
            
            cam_id = config["left_camera"] if self.side == 'left' else config["right_camera"]
            print(f"[{self.side}] Starting Gripper Vision for Cam {cam_id} at 100 FPS")
            
            # Note: rpicam-vid might need full path depending on docker environment
            # We use chroot to the mounted host filesystem so we use the host's native rpicam-vid
            cmd = [
                'chroot', '/host_root',
                'rpicam-vid', '--camera', str(cam_id), '-t', '0', '--inline', '-o', '-',
                '--width', '320', '--height', '240', '--codec', 'mjpeg', '--nopreview',
                '--denoise', 'cdn_off', '--framerate', '100', '--hflip', '1', '--vflip', '1'
            ]
            
            try:
                proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=10**6)
            except FileNotFoundError:
                print(f"ERROR: 'chroot' or 'rpicam-vid' not found. Ensure /host_root is mounted.")
                self.running = False
                break

            buffer = b''
            
            try:
                while self.running:
                    chunk = proc.stdout.read(8192)
                    if not chunk: break
                    
                    buffer += chunk
                    
                    frames = []
                    while True:
                        start = buffer.find(b'\xff\xd8')
                        if start == -1: break
                        end = buffer.find(b'\xff\xd9', start)
                        if end == -1: break
                        frames.append(buffer[start:end+2])
                        buffer = buffer[end+2:]
                        
                    if len(buffer) > 100000:
                        buffer = buffer[-10000:]
                        
                    if frames:
                        jpg_data = frames[-1] # Process only the most recent frame to kill latency
                        frame = cv2.imdecode(np.frombuffer(jpg_data, dtype=np.uint8), cv2.IMREAD_COLOR)
                        
                        if frame is not None:
                            # Detection Logic
                            aoi = aois.get(self.side, [0, 0, 320, 240])
                            # If nested in config
                            if isinstance(aoi, dict): aoi = aois['aoi'].get(self.side, [0, 0, 320, 240])
                            
                            ax, ay, aw, ah = aoi
                            
                            # Detect Dark Objects
                            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                            # Target low brightness (Value < 100)
                            mask = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 100]))
                            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                            
                            best_ratio = 0.0
                            target_found = False
                            
                            for cnt in contours:
                                area = cv2.contourArea(cnt)
                                if area < 500: continue 
                                
                                ox, oy, ow, oh = cv2.boundingRect(cnt)
                                
                                # Calculate Intersection
                                ix = max(ox, ax)
                                iy = max(oy, ay)
                                iw = min(ox + ow, ax + aw) - ix
                                ih = min(oy + oh, ay + ah) - iy
                                
                                if iw > 0 and ih > 0:
                                    ratio = (iw * ih) / (ow * oh)
                                    if ratio > best_ratio: best_ratio = ratio
                                    if ratio > 0.5: target_found = True
                            
                            self.confidence = best_ratio
                            self.detected = target_found
                            
                            # ROS Publish
                            if self.pub:
                                self.pub.publish(Bool(target_found))
                            
                            # Visual Overlays
                            if target_found:
                                cv2.rectangle(frame, (ax, ay), (ax+aw, ay+ah), (0, 255, 0), 2)
                                cv2.putText(frame, "PICK READY", (ax, ay-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            else:
                                cv2.rectangle(frame, (ax, ay), (ax+aw, ay+ah), (255, 255, 255), 1)

                            # Re-encode for stream
                            _, encoded_img = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                            
                            with self.condition:
                                self.current_frame = encoded_img.tobytes()
                                self.condition.notify_all()
            finally:
                proc.terminate()
                time.sleep(1)

    def get_frames(self):
        while self.running:
            with self.condition:
                self.condition.wait()
                frame = self.current_frame
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

# Global Broadcasters
broadcaster_left = GripperVisionBroadcaster('left')
broadcaster_right = GripperVisionBroadcaster('right')

@app.route('/stream/left')
def stream_left():
    return Response(broadcaster_left.get_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/stream/right')
def stream_right():
    return Response(broadcaster_right.get_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/status')
def status():
    return jsonify({
        "left": {"detected": broadcaster_left.detected, "confidence": round(broadcaster_left.confidence, 2)},
        "right": {"detected": broadcaster_right.detected, "confidence": round(broadcaster_right.confidence, 2)}
    })

@app.route('/')
def index():
    return """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Gripper Vision System</title>
        <style>
            body { background: #000; color: #fff; font-family: sans-serif; margin: 0; }
            .container { display: flex; width: 100vw; height: 100vh; flex-direction: column; }
            .feeds { display: flex; flex: 1; }
            .feed { flex: 1; border: 1px solid #222; position: relative; }
            img { width: 100%; height: 100%; object-fit: contain; }
            .overlay { position: absolute; top: 20px; left: 20px; font-size: 24px; font-weight: bold; }
            .pick { color: #0f0; display: none; }
            .status-bar { height: 60px; background: #111; display: flex; align-items: center; justify-content: space-around; border-top: 1px solid #333; }
            .detected { color: #0f0; font-weight: bold; animation: blink 0.5s infinite; }
            @keyframes blink { 0% { opacity: 1; } 50% { opacity: 0.3; } }
        </style>
    </head>
    <body>
        <div class="container">
            <div class="feeds">
                <div class="feed">
                    <img src="/stream/left">
                    <div id="text-left" class="overlay">LEFT</div>
                </div>
                <div class="feed">
                    <img src="/stream/right">
                    <div id="text-right" class="overlay">RIGHT</div>
                </div>
            </div>
            <div class="status-bar">
                <div id="label-left">Left: Ready</div>
                <div id="master">SYSTEM ACTIVE</div>
                <div id="label-right">Right: Ready</div>
            </div>
        </div>
        <script>
            setInterval(() => {
                fetch('/status').then(r => r.json()).then(data => {
                    document.getElementById('label-left').innerHTML = `Left: ${Math.round(data.left.confidence*100)}%`;
                    document.getElementById('label-right').innerHTML = `Right: ${Math.round(data.right.confidence*100)}%`;
                    
                    if (data.left.detected || data.right.detected) {
                        document.getElementById('master').innerHTML = "PICK AUTHORIZED";
                        document.getElementById('master').className = "detected";
                    } else {
                        document.getElementById('master').innerHTML = "SYSTEM READY";
                        document.getElementById('master').className = "";
                    }
                });
            }, 100);
        </script>
    </body>
    </html>
    """

def signal_handler(sig, frame):
    broadcaster_left.running = False
    broadcaster_right.running = False
    sys.exit(0)

if __name__ == "__main__":
    if rospy:
        rospy.init_node('gripper_vision_system', anonymous=False)
        rospy.loginfo("Gripper Vision ROS Node Started")

    signal.signal(signal.SIGINT, signal_handler)
    
    broadcaster_left.start()
    broadcaster_right.start()
    
    print("Starting Gripper Vision Service on http://0.0.0.0:5000")
    # Turn off flask logging to keep terminal clean for ROS
    import logging
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)
    
    app.run(host='0.0.0.0', port=5000, threaded=True, use_reloader=False)
