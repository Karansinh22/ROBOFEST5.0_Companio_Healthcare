# Setup — Raspberry Pi 5 + Docker

Companio runs ROS Noetic inside a Docker container on a Raspberry Pi 5 (64-bit Raspberry Pi OS). The catkin workspace (this repository) lives on the host and is bind-mounted into the container at `/root/companio_ws`, so you edit code on the host and build/run it inside the container.

## 1. Host preparation (once)

```bash
git clone https://github.com/Karansinh22/ROBOFEST5.0_Companio_Healthcare.git companio_ws
cd companio_ws

./scripts/install_docker.sh              # Docker Engine; log out and back in afterwards
sudo ./scripts/install_udev_rules.sh     # /dev/rplidar, /dev/motor_left, /dev/motor_right
sudo raspi-config                        # Interface Options -> enable I2C (arms) and the camera if used
```

Verify the serial devices after plugging in the USB hub:

```bash
ls -l /dev/rplidar /dev/motor_left /dev/motor_right
python3 scripts/identify_motors.py       # probes each ttyUSB with Modbus to confirm which motor is which
```

If the hub is plugged into a different port the `ID_PATH` values in `config/udev/99-companio.rules` need updating (`udevadm info -a -n /dev/ttyUSB0 | grep ID_PATH`).

## 2. Build the image and the workspace

```bash
./scripts/setup_container.sh
```

The script

1. builds `companio/ros-noetic:latest` from `docker/Dockerfile` (first run only, ~30–60 min on a Pi 5),
2. starts the container `companio` with `--net=host --privileged`, `/dev`, `/sys`, I2C, DRI, Hailo and audio devices mapped, X11 forwarded and the workspace mounted,
3. runs `catkin_make` inside the container.

Use `./scripts/setup_container.sh --rebuild` to recreate the container and rebuild the image. The container is started with `--restart unless-stopped`, so it survives reboots.

## 3. Everyday commands

| Script | What it does |
|---|---|
| `./scripts/enter_container.sh` | shell inside the container with ROS and the workspace sourced |
| `./scripts/run.sh <cmd>` | run one ROS command inside the container, e.g. `./scripts/run.sh rostopic list` |
| `./scripts/build_workspace.sh` | `catkin_make` after code changes |
| `./scripts/check_status.sh` | container / devices / packages health check |
| `./scripts/start_teleop.sh` | hardware + keyboard teleop |
| `./scripts/start_mapping.sh` | SLAM mapping + rooms + RViz + dashboard |
| `./scripts/save_map.sh <name>` | save the current map |
| `./scripts/start_navigation.sh` | autonomous navigation + RViz + dashboard |
| `./scripts/start_rviz.sh [mapping\|navigation]` | RViz only |
| `./scripts/start_arms.sh` | arm + tower controllers |
| `./scripts/locations.sh` | room console (save / go / delete …) |
| `./scripts/stop_container.sh` | stop the container |

Launch arguments are passed straight through, e.g. `./scripts/start_navigation.sh map_name:=ward_b rviz:=false`.

## 4. Working inside the container manually

```bash
docker exec -it companio bash
# ROS and the workspace are sourced automatically by ~/.bashrc
roslaunch companio_bringup navigation.launch
```

Multiple terminals: run `docker exec -it companio bash` in each one. `roscore` is started automatically by the first `roslaunch`.

## 5. RViz on the Pi

RViz runs inside the container and is displayed on the Pi's screen through X11 (`xhost +local:docker` is executed by the setup script). If a display is not attached, launch with `rviz:=false` and use the web dashboard instead, or run RViz on a laptop with `ROS_MASTER_URI=http://<robot-ip>:11311`.

## 6. Updating the code

```bash
git pull
./scripts/build_workspace.sh
```
