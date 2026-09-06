# Web dashboard

The dashboard (`src/companio_web/www/index.html`) is a single static page that talks to ROS through **rosbridge** (websocket, port 9090). It is served by `web_server.py` on port **8000** and is started automatically by `mapping.launch` and `navigation.launch` (`web:=false` disables it), or on its own:

```bash
roslaunch companio_web web.launch
```

Open `http://<robot-ip>:8000` from the on-board touchscreen (`http://localhost:8000`), a phone or a laptop on the same network. The page connects to `ws://<page-host>:9090`; a different bridge can be given in the URL box or with `?ros=ws://10.0.0.5:9090`.

## Panels

| Panel | Function |
|---|---|
| **Map** | occupancy grid, robot (yellow arrow), laser (orange), global plan (green), rooms (blue). Wheel = zoom, *Fit* resets. Modes: **Navigate** (click; drag for heading), **Set Pose** (AMCL initial pose), **Save Room** (click, then name it), **Pan**. Clicking a room marker opens *Go here / Robot is here / Rename / Delete*. |
| **Navigation** | state badge (`idle / navigating / arrived / failed / cancelled`), target, distance left; **Cancel**, **Go Home** |
| **Rooms** | list with **Go** / delete; click a name to rename; *Save current pose* |
| **Manual drive** | hold the arrows (or W/A/S/D), speed sliders; releasing publishes a stop |
| **Telemetry** | pose, velocity, nearest obstacle, laser rate, move_base state, uptime |
| **Arms & tower** | joint sliders for both arms and the pan/tilt tower, *Home* presets (publishes `Float64MultiArray` to the controllers) |
| **Camera** | live `/camera/color/image_raw/compressed` stream (needs `camera.launch`) |
| **E-STOP** | zeroes `/cmd_vel` repeatedly and cancels every move_base goal |

## Topics and services used

Subscribes: `/map` (png-compressed), `/robot_pose`, `/scan`, `/odom`, `/move_base/NavfnROS/plan`, `/locations/list`, `/navigation/status`, `/move_base/status`, `/camera/color/image_raw/compressed`.

Publishes: `/cmd_vel`, `/move_base_simple/goal`, `/initialpose`, `/move_base/cancel`, `/left_arm/joint_commands`, `/right_arm/joint_commands`, `/tower/joint_commands`.

Services: `/locations/save`, `/locations/go_to`, `/locations/delete`, `/locations/rename`, `/locations/localize_at`, `/locations/cancel`.

## Notes

* `roslib.min.js` is bundled in `www/vendor/`, so the page works without internet access.
* The laser overlay assumes the lidar is mounted rotated 180° (`LASER_YAW_OFFSET` in the page) — matches the URDF.
* For a remote laptop, allow ports 8000 and 9090 through any firewall on the Pi.
