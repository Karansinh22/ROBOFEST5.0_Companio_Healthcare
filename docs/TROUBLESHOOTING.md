# Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `Cannot open motor controllers` / `No such file: /dev/motor_left` | udev rules not installed or hub on a different port. `sudo ./scripts/install_udev_rules.sh`, then `python3 scripts/identify_motors.py`. Re-run `./scripts/setup_container.sh` after re-plugging so the device nodes are refreshed inside the container. |
| Wrong wheel responds / robot turns instead of driving straight | motors swapped: check slave IDs (left = 2, right = 5) with `identify_motors.py`; test each wheel with `rosrun companio_base wheel_test.py`. |
| `Input/output error` on a serial port, or `package companio_bringup not found` | the USB hub reconnected and the container lost the mount. `./scripts/setup_container.sh --rebuild`. |
| RPLidar: `Error, cannot bind to the specified serial port` | `/dev/rplidar` missing or in use. `ls -l /dev/rplidar`, unplug/replug, check no second `rplidarNode` is running. |
| No `/map` while mapping | `rostopic hz /scan` and `rostopic echo /odom` must both be alive; check the TF tree `rosrun tf view_frames`. |
| AMCL particles never converge | wrong initial pose or the map does not match the room; give a better *2D Pose Estimate* and drive a metre. |
| move_base: `Failed to get a plan` | goal inside an obstacle / inflation; lower `inflation_radius` or pick a free spot. |
| Robot oscillates near the goal | reduce `max_vel_theta` or increase `xy_goal_tolerance` in `dwa_local_planner.yaml`. |
| Dashboard shows "disconnected" | rosbridge not running (`web:=true` or `roslaunch companio_web web.launch`), wrong host in the URL box, or port 9090 blocked. |
| Dashboard map is empty | no `/map` publisher (start mapping or navigation). |
| `location_manager` says *Robot pose not available* | no `map → base_footprint` transform yet: GMapping/AMCL not up, or lidar/odometry missing. |
| Rooms disappeared | they belong to the map name used at launch: `map_name:=<same name as when mapping>`. Files: `src/companio_description/maps/<map>.locations.yaml`. |
| RViz does not open inside the container | run `xhost +local:docker` on the host, make sure `DISPLAY` is set, or launch with `rviz:=false` and use the dashboard. |
| Arms: `Cannot initialize PCA9685` | enable I2C (`raspi-config`), check `ls /dev/i2c-1` and `i2cdetect -y 1` shows `40`. |
| `module 'gpiod' has no attribute 'Chip'` | the pip `gpiod` package shadows the system one: `pip3 uninstall gpiod`; use `python3-libgpiod` (already in the image). |
| Voice: `arecord: device not found` | `arecord -l`, pass `_audio_device:=plughw:X,0`. Google recognition needs internet. |
| RealSense not found | USB 3 port/cable and a strong power supply; `lsusb | grep -i realsense`. |
| Build errors after editing messages | `./scripts/build_workspace.sh` (message generation) and re-source: `source devel/setup.bash`. |

Useful checks inside the container:

```bash
rostopic list
rostopic hz /scan /odom
rosrun tf view_frames && evince frames.pdf
rosservice list | grep locations
rosnode info /location_manager
```
