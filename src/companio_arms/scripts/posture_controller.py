#!/usr/bin/env python3
"""
posture_controller.py
=====================
Contract / expand the robot between its working posture and its 30x30x30 cm
rest posture by sequencing the two arm controllers and the tower controller.

  contract : both arms fold upward, then the tower rotates so the folded arms
             sit over the base
  expand   : the tower rotates back to the front, then the arms deploy

Interfaces
  /posture/contract   std_srvs/Trigger
  /posture/expand     std_srvs/Trigger
  /posture/command    std_msgs/String   "contract" | "expand"   (voice, dashboard)
  /posture/state      std_msgs/String   latched: unknown | expanded | contracted |
                                        contracting | expanding | error

Requires left_arm_controller, right_arm_controller and tower_controller
(roslaunch companio_arms arms.launch).  Postures live in config/postures.yaml.
"""

import os
import threading
import time

import rospkg
import rospy
import yaml
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger, TriggerResponse

ARM_JOINTS = ["shoulder", "arm", "elbow", "gripper"]
TOWER_JOINTS = ["tower", "camera"]


class PostureController:
    def __init__(self):
        rospy.init_node("posture_controller")
        default_cfg = os.path.join(rospkg.RosPack().get_path("companio_arms"), "config", "postures.yaml")
        cfg_path = rospy.get_param("~config", default_cfg)
        with open(cfg_path) as f:
            cfg = yaml.safe_load(f)
        self.postures = cfg["postures"]
        self.settle = float(cfg.get("settle_time", 1.0))
        self.deg_per_s = float(cfg.get("servo_deg_per_s", 20.0))
        self.tolerance = float(cfg.get("tolerance_deg", 4.0))
        self.stage_timeout = float(cfg.get("stage_timeout", 25.0))

        self.pubs = {
            "left": rospy.Publisher("/left_arm/joint_commands", Float64MultiArray, queue_size=1),
            "right": rospy.Publisher("/right_arm/joint_commands", Float64MultiArray, queue_size=1),
            "tower": rospy.Publisher("/tower/joint_commands", Float64MultiArray, queue_size=1),
        }
        self.state_pub = rospy.Publisher("/posture/state", String, queue_size=1, latch=True)
        self.busy_pubs = {
            "left": rospy.Publisher("/left_arm/is_moving", Bool, queue_size=1, latch=True),
            "right": rospy.Publisher("/right_arm/is_moving", Bool, queue_size=1, latch=True),
        }

        self.joints = {}          # joint name -> current angle (from the controllers)
        self.joints_lock = threading.Lock()
        rospy.Subscriber("/left_arm/joint_states", JointState, self._on_joint_state)
        rospy.Subscriber("/right_arm/joint_states", JointState, self._on_joint_state)
        rospy.Subscriber("/tower/joint_states", JointState, self._on_joint_state)
        rospy.Subscriber("/posture/command", String, self._on_command, queue_size=1)
        rospy.Service("/posture/contract", Trigger, lambda _r: self._trigger("contracted"))
        rospy.Service("/posture/expand", Trigger, lambda _r: self._trigger("expanded"))

        self.lock = threading.Lock()
        self.state = "unknown"
        self._publish_state()
        rospy.loginfo("Posture controller ready (config: %s)", cfg_path)

    # --------------------------------------------------------------- state --
    def _publish_state(self):
        self.state_pub.publish(String(data=self.state))

    def _set_state(self, state):
        self.state = state
        self._publish_state()
        rospy.loginfo("Posture: %s", state)

    def _on_joint_state(self, msg):
        with self.joints_lock:
            for name, pos in zip(msg.name, msg.position):
                self.joints[name] = float(pos)

    # ------------------------------------------------------------ commands --
    def _on_command(self, msg):
        cmd = msg.data.strip().lower()
        if cmd in ("contract", "contracted", "fold", "rest", "compact"):
            self._trigger("contracted")
        elif cmd in ("expand", "expanded", "unfold", "deploy", "open"):
            self._trigger("expanded")
        else:
            rospy.logwarn("Unknown posture command '%s' (use contract | expand)", msg.data)

    def _trigger(self, target):
        if not self.lock.acquire(blocking=False):
            return TriggerResponse(False, "Posture change already in progress")
        try:
            if self.state == target:
                return TriggerResponse(True, "Already %s" % target)
            threading.Thread(target=self._run, args=(target,), daemon=True).start()
            return TriggerResponse(True, "%s started" % ("Contracting" if target == "contracted" else "Expanding"))
        finally:
            self.lock.release()

    # ------------------------------------------------------------ sequence --
    def _run(self, target):
        with self.lock:
            posture = self.postures[target]
            self._set_state("contracting" if target == "contracted" else "expanding")
            for side in ("left", "right"):
                self.busy_pubs[side].publish(Bool(data=True))
            try:
                if target == "contracted":
                    stages = [("arms", {"left": posture["left"], "right": posture["right"]}),
                              ("tower", {"tower": posture["tower"]})]
                else:
                    stages = [("tower", {"tower": posture["tower"]}),
                              ("arms", {"left": posture["left"], "right": posture["right"]})]
                for label, goals in stages:
                    if not self._move_stage(label, goals):
                        self._set_state("error")
                        return
                self._set_state(target)
            finally:
                for side in ("left", "right"):
                    self.busy_pubs[side].publish(Bool(data=False))

    def _move_stage(self, label, goals):
        rospy.loginfo("Posture stage: %s -> %s", label, goals)
        max_delta = 0.0
        for group, angles in goals.items():
            names = TOWER_JOINTS if group == "tower" else ARM_JOINTS
            prefix = "tower_" if group == "tower" else group + "_"
            with self.joints_lock:
                for name, angle in zip(names, angles):
                    current = self.joints.get(prefix + name)
                    if current is not None:
                        max_delta = max(max_delta, abs(current - angle))
            if self.pubs[group].get_num_connections() == 0:
                rospy.logerr("No %s controller listening on %s - start companio_arms arms.launch",
                             group, self.pubs[group].resolved_name)
                return False
            self.pubs[group].publish(Float64MultiArray(data=[float(a) for a in angles]))

        # Wait for the controllers to arrive: joint feedback when available, else travel-time estimate.
        estimate = (max_delta if max_delta > 0 else 180.0) / self.deg_per_s + self.settle
        deadline = time.time() + min(self.stage_timeout, max(estimate, 2.0) + 5.0)
        while not rospy.is_shutdown() and time.time() < deadline:
            if self._reached(goals):
                time.sleep(self.settle)
                return True
            time.sleep(0.1)
        if self._reached(goals):
            return True
        rospy.logwarn("Stage %s did not report arrival in time (continuing)", label)
        return True

    def _reached(self, goals):
        with self.joints_lock:
            for group, angles in goals.items():
                names = TOWER_JOINTS if group == "tower" else ARM_JOINTS
                prefix = "tower_" if group == "tower" else group + "_"
                for name, angle in zip(names, angles):
                    current = self.joints.get(prefix + name)
                    if current is None or abs(current - angle) > self.tolerance:
                        return False
        return True


if __name__ == "__main__":
    try:
        PostureController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
