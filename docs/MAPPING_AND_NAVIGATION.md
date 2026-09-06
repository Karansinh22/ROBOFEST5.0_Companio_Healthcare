# Mapping, rooms and navigation

## Concepts

* A **map** is a pair `<name>.pgm/<name>.yaml` in `src/companio_description/maps/`.
* The **rooms** (named locations) of a map are stored next to it in `<name>.locations.yaml` and managed by the `location_manager` node. They are poses in the `map` frame, so a room saved during mapping is valid later during navigation with the saved map.
* `location_manager` runs in both mapping and navigation sessions. During mapping, navigation services are disabled (no move_base), everything else works.

## 1. Mapping

```bash
./scripts/start_mapping.sh map_name:=hospital_map
```

Starts: motors, odometry, lidar, URDF, GMapping, location manager, keyboard teleop (this terminal), RViz (mapping view) and the web dashboard.

Driving tips: ≤ 0.2 m/s, avoid fast rotations, overlap your paths and return to the start for loop closure.

### Saving rooms while mapping

| Where | How |
|---|---|
| RViz | select the **Publish Point** tool and click on the floor. A blue disc with a label appears. Drag the disc to move it, use the ring to rotate it, right-click for the menu. |
| Dashboard | **Rooms → Save current pose** (type a name first) or **Save Room** map mode → click the spot |
| Terminal | `./scripts/locations.sh save "Ward A"` |
| Voice | `roslaunch companio_voice voice.launch` → *"save this as Ward A"* |

Rooms saved from RViz/point clicks get automatic names (`Location 1`, `Location 2` …). Rename them from the dashboard (click the name) or with `./scripts/locations.sh rename "Location 1" "Ward A"`.

### Saving the map

```bash
./scripts/save_map.sh hospital_map
```

Do this while the mapping session is still running. The rooms file is written live on every change, nothing extra to do.

## 2. Navigation

```bash
./scripts/start_navigation.sh map_name:=hospital_map
```

Starts: hardware, `map_server`, AMCL, move_base (DWA), location manager (navigation enabled), RViz (navigation view) and the dashboard.

1. **Localise**: RViz *2D Pose Estimate*, dashboard *Set Pose* mode, or `./scripts/locations.sh localize Home` if the robot starts in a saved room. The AMCL particle cloud should collapse onto the robot after a short drive.
2. **Send a goal**: RViz *2D Nav Goal*, right-click a room marker → *Go here*, dashboard **Go**, `./scripts/locations.sh go "Ward A"`, or voice *"go to Ward A"*.
3. Watch `/navigation/status` (dashboard "Navigation" panel): `navigating` → `arrived` / `failed` / `cancelled`. **Cancel** stops the current goal; **E-STOP** on the dashboard also zeroes `/cmd_vel`.

## 3. Room console

```
./scripts/locations.sh
companio> list
companio> save Ward A
companio> go ward a          # fuzzy matching
companio> rename "Location 1" "Pharmacy"
companio> localize Home
companio> cancel
companio> status
```

One-shot: `./scripts/locations.sh go "Ward A"`.

## 4. Services and topics

| Name | Type | Description |
|---|---|---|
| `/locations/save` | companio_msgs/SaveLocation | `name` ("" = auto), `use_robot_pose`, `pose` (x, y, theta) |
| `/locations/go_to`, `/locations/delete`, `/locations/localize_at` | companio_msgs/LocationName | by name |
| `/locations/rename` | companio_msgs/RenameLocation | |
| `/locations/cancel`, `/locations/reload` | std_srvs/Trigger | |
| `/locations/list` | std_msgs/String (JSON, latched) | `{map, locations:[{name,x,y,yaw}]}` |
| `/locations/markers` | visualization_msgs/MarkerArray | arrows + labels |
| `/locations/update` | interactive markers | RViz editing |
| `/locations/add_pose` | geometry_msgs/PoseStamped | save from any pose publisher |
| `/clicked_point` | geometry_msgs/PointStamped | RViz Publish Point |
| `/navigation/status` | std_msgs/String (JSON, latched) | `{state, target, message, distance}` |
| `/robot_pose` | geometry_msgs/PoseStamped | 5 Hz, map frame |

Example from the command line:

```bash
rosservice call /locations/save "{name: 'Ward A', use_robot_pose: true, pose: {x: 0, y: 0, theta: 0}}"
rosservice call /locations/go_to "name: 'Ward A'"
```

## 5. Tuning

| File | Purpose |
|---|---|
| `companio_navigation/config/gmapping.yaml` | SLAM: range, particles, map extent |
| `companio_navigation/config/amcl.yaml` | localisation: particle count, odometry noise |
| `companio_navigation/config/costmap_common_params.yaml` | footprint, inflation radius (0.15 m — the robot is narrow) |
| `companio_navigation/config/dwa_local_planner.yaml` | speeds (0.3 m/s, 0.5 rad/s), goal tolerances |
| `companio_navigation/launch/move_base.launch` | recovery behaviours are disabled on purpose |
| `companio_base/config/base.yaml` | odometry calibration |

Verify odometry before trusting a map: drive 1 m forward and rotate 360°, compare `/odom` with reality; adjust `ticks_meter` / `base_width` accordingly.
