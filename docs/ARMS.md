# Arms and manipulation

<img src="images/companio_arms_reach.jpg" width="300" align="right">

Two 4-DOF arms (shoulder, arm, elbow, gripper) and a pan/tilt camera tower are driven by a PCA9685 PWM board on I2C bus 1 (address 0x40). Each controller

* clamps every joint to its safe range,
* applies collision-avoidance rules between joints,
* moves joints sequentially in 5° steps with smooth interpolation,
* publishes `sensor_msgs/JointState` on `/<side>_arm/joint_states`,
* returns to the home pose on shutdown.

## Start

```bash
./scripts/start_arms.sh                       # left + right + tower (args: left:=false right:=false tower:=false)
./scripts/run.sh rosrun companio_arms arm_teleop.py
```

Teleop keys: `1`/`2` select arm · `w/s` shoulder · `e/d` arm in/out · `r/f` elbow · `y/h` gripper · `q` quit.
Tower: `rosrun companio_arms tower_teleop.py` (`a/d` pan, `w/s` tilt, `h` home).

Joint command topics take degrees in the order `[shoulder, arm, elbow, gripper]`:

```bash
rostopic pub -1 /left_arm/joint_commands std_msgs/Float64MultiArray "data: [80, 85, 95, 160]"
rostopic pub -1 /tower/joint_commands std_msgs/Float64MultiArray "data: [90, 90]"
```

## Contract / expand (rest posture)

The robot's rest posture fits inside 30 × 30 × 30 cm: both arms fold upward and the tower rotates 180° so the folded arms sit over the base. `posture_controller.py` (started by `arms.launch`) sequences the three controllers:

* **contract** — arms to the folded pose, wait, tower to the rest angle
* **expand** — tower back to the front, wait, arms to the working pose

| Trigger | Command |
|---|---|
| service | `rosservice call /posture/contract` / `rosservice call /posture/expand` |
| topic | `rostopic pub -1 /posture/command std_msgs/String "data: contract"` |
| keyboard | `arm_teleop.py`: `c` contract, `x` expand |
| dashboard | *Arms & tower* → **Contract** / **Expand** |
| voice | "contract" / "fold" / "rest position", "expand" / "unfold" / "deploy" |

`/posture/state` (latched) reports `expanded`, `contracted`, `contracting`, `expanding` or `error`. The joint angles of both postures and the timing are in [`config/postures.yaml`](../src/companio_arms/config/postures.yaml); tune them with the teleop tools and paste the values there. While a posture change runs, `/left_arm/is_moving` and `/right_arm/is_moving` are set so the pick sequences stay idle.

## Inverse kinematics

| Node | Method | Input |
|---|---|---|
| `right_arm_ik_controller.py` / `left_arm_ik_controller.py` | analytic 3-link IK with no-go zones (base, tower, camera) | `/<side>_arm/move_to_xyz` (cm, robot frame) |
| `right_arm_numerical_ik.py` | brute-force numeric search on the forward model | same |
| `autonomous_arm_teleop.py`, `autonomous_left_arm_teleop.py` | terminal XYZ prompt that publishes to the IK nodes | keyboard |

```bash
rosrun companio_arms right_arm_controller.py
rosrun companio_arms right_arm_ik_controller.py
rosrun companio_arms autonomous_arm_teleop.py      # enter x y z in cm
```

## Pick sequences and gripper vision

`left_arm_sequence.py` / `right_arm_sequence.py` run a 5-stage reach-grasp-lift sequence (`H`) and a gripper cycle (`D`). They can be triggered automatically by `gripper_vision_node.py`, which watches the two gripper cameras (area-of-interest in `scripts/aoi_config.json`) and publishes `/gripper_vision/<side>/target_detected`. The two arms coordinate through `/<side>_arm/is_moving` so they never move at the same time.

```bash
roslaunch companio_arms autonomous_pick.launch
rosrun companio_arms left_arm_sequence.py
rosrun companio_arms right_arm_sequence.py
```

The vision node also exposes an MJPEG preview and AOI editor on `http://<robot-ip>:5000`.
