# Line following and zone detection

Besides free navigation on a map, Companio can follow floor lines — useful in corridors marked with tape.

## IR sensor line following (GPIO)

Five IR reflectance sensors on BCM pins 17, 27, 22, 23, 24 (left → right). Requires the hardware bring-up (`./scripts/start_teleop.sh` with the keyboard idle, or `roslaunch companio_bringup base.launch`).

| Script | Behaviour |
|---|---|
| `line_following.py` | follow a white line on a dark floor |
| `line_following_black.py` | follow a black line on a light floor |
| `line_path_following.py` | black line + junction detection + fixed turn sequence to reach a room (`_target_room:=ROOM_1` or `_turn_sequence:="LEFT,RIGHT,STRAIGHT"`) |
| `keyboard_line_following.py` | follow the line and stop at location patches; keys `1-4` go to a location, `H` home, `S` stop, `C` continue |
| `voice_line_following.py` | same as above, driven by voice ("location 2", "home", "stop") |

```bash
./scripts/run.sh rosrun companio_line_follower line_following_black.py _debug:=true
./scripts/run.sh rosrun companio_line_follower keyboard_line_following.py
```

All GPIO scripts publish `/cmd_vel` and stop the robot when the line is lost or on shutdown.

## Camera line following (RealSense, optional Hailo NPU)

| Launch | Pipeline |
|---|---|
| `ai_line_follower.launch` | camera → `hailo_line_detector_node` (centroid of the tape, groove rejection) → PID `line_follower_node` → `motor_control_node` → motors |
| `ai_line_follower_direct.launch` | all-in-one follower with time-based 90° corner handling |
| `path_following_direct.launch` | follows the mid-line between two tape lines |
| `ai_destination_follower.launch` | line following + **QR codes at junctions**: each code carries a JSON route table, the robot turns towards the requested destination (`destination:="Room A"`) and avoids red zones |

```bash
./scripts/run.sh roslaunch companio_line_follower ai_destination_follower.launch destination:="Room A"
```

The Hailo detector falls back to CPU OpenCV automatically when `/dev/hailo0` is absent.

## Zone detection

`companio_zone_detection` segments blue (safe path) and red (infection zone) floor markings in HSV space and publishes `/safe_detected`, `/infection_detected` plus the masks.

```bash
./scripts/run.sh roslaunch companio_zone_detection zone_detection.launch
./scripts/run.sh rosrun companio_zone_detection find_camera.py     # locate the /dev/video device
```
