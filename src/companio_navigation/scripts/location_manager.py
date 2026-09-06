#!/usr/bin/env python3
"""
location_manager.py
===================
Named-location ("room") manager for the Companio healthcare robot.

Locations are poses in the map frame.  They can be recorded while a map is
being built (GMapping) or later during navigation (AMCL) and are persisted to
a YAML file that lives next to the map (<map_name>.locations.yaml).

Interfaces
----------
Services (companio_msgs)
  /locations/save         SaveLocation    save the robot pose (or a given pose) under a name
  /locations/delete       LocationName    delete a location
  /locations/rename       RenameLocation  rename a location
  /locations/go_to        LocationName    send move_base to a location (non-blocking)
  /locations/localize_at  LocationName    publish /initialpose at a location (AMCL reset)
  /locations/cancel       std_srvs/Trigger cancel the active navigation goal
  /locations/reload       std_srvs/Trigger reload the YAML file from disk

Topics
  /locations/list      std_msgs/String (latched JSON)         all locations
  /locations/markers   visualization_msgs/MarkerArray (latched) arrows + labels for RViz / dashboards
  /locations/update    interactive markers (drag to move, right-click menu: Go / Delete / Move here)
  /locations/add_pose  geometry_msgs/PoseStamped (in)          save a location from a pose (auto-named)
  /clicked_point       geometry_msgs/PointStamped (in)         RViz "Publish Point" -> save a location
  /robot_pose          geometry_msgs/PoseStamped               robot pose in the map frame (5 Hz)
  /navigation/status   std_msgs/String (latched JSON)         idle | navigating | arrived | failed | cancelled
"""

import json
import math
import os
import threading

import yaml

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import (Point, PointStamped, Pose, PoseStamped,
                               PoseWithCovarianceStamped, Quaternion, Vector3)
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import (InteractiveMarker, InteractiveMarkerControl,
                                    InteractiveMarkerFeedback, Marker, MarkerArray)

from companio_msgs.srv import (LocationName, LocationNameResponse, RenameLocation,
                               RenameLocationResponse, SaveLocation, SaveLocationResponse)


def yaw_to_quaternion(yaw):
    return Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quaternion_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class LocationStore:
    """Ordered, name-keyed set of (x, y, yaw) poses persisted as YAML."""

    def __init__(self, path, map_name, frame_id):
        self.path = path
        self.map_name = map_name
        self.frame_id = frame_id
        self.locations = []  # list of dicts: name, x, y, yaw
        self.load()

    # -- persistence -----------------------------------------------------------
    def load(self):
        self.locations = []
        if not os.path.exists(self.path):
            rospy.loginfo("No locations file yet at %s (will be created on first save)", self.path)
            return
        try:
            with open(self.path, "r") as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:
            rospy.logerr("Cannot read %s: %s", self.path, exc)
            return
        for item in data.get("locations", []) or []:
            try:
                self.locations.append({
                    "name": str(item["name"]),
                    "x": float(item["x"]),
                    "y": float(item["y"]),
                    "yaw": float(item.get("yaw", 0.0)),
                })
            except (KeyError, TypeError, ValueError):
                rospy.logwarn("Skipping malformed location entry: %s", item)
        rospy.loginfo("Loaded %d location(s) from %s", len(self.locations), self.path)

    def save(self):
        data = {
            "map": self.map_name,
            "frame_id": self.frame_id,
            "locations": [dict(name=l["name"], x=round(l["x"], 4), y=round(l["y"], 4),
                               yaw=round(l["yaw"], 4)) for l in self.locations],
        }
        directory = os.path.dirname(self.path)
        if directory and not os.path.isdir(directory):
            os.makedirs(directory)
        tmp = self.path + ".tmp"
        with open(tmp, "w") as f:
            f.write("# Named locations for map '%s' (metres / radians, %s frame).\n"
                    % (self.map_name, self.frame_id))
            f.write("# Managed by companio_navigation/location_manager.py\n")
            yaml.safe_dump(data, f, sort_keys=False)
        os.replace(tmp, self.path)

    # -- queries ---------------------------------------------------------------
    def find(self, name):
        key = name.strip().lower()
        for loc in self.locations:
            if loc["name"].lower() == key:
                return loc
        return None

    def names(self):
        return [l["name"] for l in self.locations]

    def unique_name(self, prefix="Location"):
        n = len(self.locations) + 1
        while self.find("%s %d" % (prefix, n)):
            n += 1
        return "%s %d" % (prefix, n)

    def upsert(self, name, x, y, yaw):
        loc = self.find(name)
        if loc is None:
            loc = {"name": name.strip(), "x": x, "y": y, "yaw": yaw}
            self.locations.append(loc)
            created = True
        else:
            loc.update(x=x, y=y, yaw=yaw)
            created = False
        self.save()
        return loc, created

    def delete(self, name):
        loc = self.find(name)
        if loc is None:
            return False
        self.locations.remove(loc)
        self.save()
        return True

    def rename(self, old, new):
        loc = self.find(old)
        if loc is None:
            return False, "No location named '%s'" % old
        if self.find(new) and self.find(new) is not loc:
            return False, "A location named '%s' already exists" % new
        loc["name"] = new.strip()
        self.save()
        return True, "Renamed '%s' to '%s'" % (old, new)


class LocationManager:
    def __init__(self):
        rospy.init_node("location_manager")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.map_name = rospy.get_param("~map_name", "map")
        self.navigation_enabled = bool(rospy.get_param("~navigation_enabled", True))
        pose_rate = float(rospy.get_param("~pose_publish_rate", 5.0))
        default_file = os.path.join(os.path.expanduser("~"), "%s.locations.yaml" % self.map_name)
        self.locations_file = rospy.get_param("~locations_file", default_file)

        self.lock = threading.RLock()
        self.store = LocationStore(self.locations_file, self.map_name, self.map_frame)
        self.tf_listener = tf.TransformListener()

        # publishers
        self.list_pub = rospy.Publisher("/locations/list", String, queue_size=1, latch=True)
        self.markers_pub = rospy.Publisher("/locations/markers", MarkerArray, queue_size=1, latch=True)
        self.pose_pub = rospy.Publisher("/robot_pose", PoseStamped, queue_size=1)
        self.status_pub = rospy.Publisher("/navigation/status", String, queue_size=1, latch=True)
        self.initialpose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)

        # interactive markers (RViz)
        self.im_server = InteractiveMarkerServer("locations")
        self.menu = MenuHandler()
        self.menu.insert("Go here", callback=self._menu_go)
        self.menu.insert("Move to robot pose", callback=self._menu_move_here)
        self.menu.insert("Set robot pose here (localize)", callback=self._menu_localize)
        self.menu.insert("Delete", callback=self._menu_delete)

        # navigation
        self.client = None
        self.active_target = None
        self.status = {"state": "idle", "target": "", "message": "", "distance": 0.0}
        if self.navigation_enabled:
            self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
            threading.Thread(target=self._wait_for_move_base, daemon=True).start()

        # services
        rospy.Service("/locations/save", SaveLocation, self.srv_save)
        rospy.Service("/locations/delete", LocationName, self.srv_delete)
        rospy.Service("/locations/rename", RenameLocation, self.srv_rename)
        rospy.Service("/locations/go_to", LocationName, self.srv_go_to)
        rospy.Service("/locations/localize_at", LocationName, self.srv_localize_at)
        rospy.Service("/locations/cancel", Trigger, self.srv_cancel)
        rospy.Service("/locations/reload", Trigger, self.srv_reload)

        # topic inputs
        rospy.Subscriber("/locations/add_pose", PoseStamped, self.on_add_pose, queue_size=1)
        rospy.Subscriber("/clicked_point", PointStamped, self.on_clicked_point, queue_size=1)

        self.publish_all()
        self.publish_status()
        rospy.Timer(rospy.Duration(1.0 / max(pose_rate, 0.5)), self.on_pose_timer)
        rospy.loginfo("Location manager ready: map='%s', file=%s, navigation=%s",
                      self.map_name, self.locations_file, self.navigation_enabled)

    # ------------------------------------------------------------------ pose --
    def robot_pose(self):
        """Robot pose (x, y, yaw) in the map frame, or None if TF is not available."""
        try:
            self.tf_listener.waitForTransform(self.map_frame, self.base_frame,
                                              rospy.Time(0), rospy.Duration(0.5))
            (trans, rot) = self.tf_listener.lookupTransform(self.map_frame, self.base_frame,
                                                            rospy.Time(0))
        except (tf.Exception, tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            return None
        yaw = tf.transformations.euler_from_quaternion(rot)[2]
        return trans[0], trans[1], yaw

    def on_pose_timer(self, _event):
        pose = self.robot_pose()
        if pose is None:
            return
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.map_frame
        msg.pose.position = Point(pose[0], pose[1], 0.0)
        msg.pose.orientation = yaw_to_quaternion(pose[2])
        self.pose_pub.publish(msg)
        if self.status["state"] == "navigating" and self.active_target:
            loc = self.store.find(self.active_target)
            if loc:
                self.status["distance"] = round(math.hypot(loc["x"] - pose[0], loc["y"] - pose[1]), 2)

    # -------------------------------------------------------------- services --
    def srv_save(self, req):
        with self.lock:
            name = req.name.strip() or self.store.unique_name()
            if req.use_robot_pose:
                pose = self.robot_pose()
                if pose is None:
                    return SaveLocationResponse(False, "Robot pose not available (no %s -> %s transform)"
                                                % (self.map_frame, self.base_frame), "")
                x, y, yaw = pose
            else:
                x, y, yaw = req.pose.x, req.pose.y, req.pose.theta
            loc, created = self.store.upsert(name, x, y, yaw)
            self.publish_all()
            verb = "Saved" if created else "Updated"
            msg = "%s '%s' at x=%.2f y=%.2f yaw=%.2f" % (verb, loc["name"], x, y, yaw)
            rospy.loginfo(msg)
            return SaveLocationResponse(True, msg, loc["name"])

    def srv_delete(self, req):
        with self.lock:
            if not self.store.delete(req.name):
                return LocationNameResponse(False, "No location named '%s'" % req.name)
            self.publish_all()
            rospy.loginfo("Deleted location '%s'", req.name)
            return LocationNameResponse(True, "Deleted '%s'" % req.name)

    def srv_rename(self, req):
        with self.lock:
            ok, msg = self.store.rename(req.old_name, req.new_name)
            if ok:
                self.publish_all()
            return RenameLocationResponse(ok, msg)

    def srv_go_to(self, req):
        ok, msg = self.go_to(req.name)
        return LocationNameResponse(ok, msg)

    def srv_localize_at(self, req):
        ok, msg = self.localize_at(req.name)
        return LocationNameResponse(ok, msg)

    def srv_cancel(self, _req):
        ok, msg = self.cancel()
        return TriggerResponse(ok, msg)

    def srv_reload(self, _req):
        with self.lock:
            self.store.load()
            self.publish_all()
        return TriggerResponse(True, "Reloaded %d location(s)" % len(self.store.locations))

    # ----------------------------------------------------------- topic inputs --
    def on_add_pose(self, msg):
        if msg.header.frame_id and msg.header.frame_id.lstrip("/") != self.map_frame:
            rospy.logwarn("Ignoring pose in frame '%s' (expected '%s')", msg.header.frame_id, self.map_frame)
            return
        with self.lock:
            name = self.store.unique_name()
            self.store.upsert(name, msg.pose.position.x, msg.pose.position.y,
                              quaternion_to_yaw(msg.pose.orientation))
            self.publish_all()
        rospy.loginfo("Saved '%s' from pose input", name)

    def on_clicked_point(self, msg):
        if msg.header.frame_id and msg.header.frame_id.lstrip("/") != self.map_frame:
            rospy.logwarn("Ignoring point in frame '%s' (expected '%s')", msg.header.frame_id, self.map_frame)
            return
        x, y = msg.point.x, msg.point.y
        pose = self.robot_pose()
        yaw = math.atan2(y - pose[1], x - pose[0]) if pose else 0.0
        with self.lock:
            name = self.store.unique_name()
            self.store.upsert(name, x, y, yaw)
            self.publish_all()
        rospy.loginfo("Saved '%s' from clicked point (x=%.2f y=%.2f)", name, x, y)

    # ------------------------------------------------------------ navigation --
    def _wait_for_move_base(self):
        rospy.loginfo("Waiting for move_base action server ...")
        while not rospy.is_shutdown():
            if self.client.wait_for_server(rospy.Duration(2.0)):
                rospy.loginfo("Connected to move_base")
                return

    def go_to(self, name):
        loc = self.store.find(name)
        if loc is None:
            return False, "No location named '%s'" % name
        if not self.navigation_enabled or self.client is None:
            return False, "Navigation is disabled in this session (mapping mode)"
        if not self.client.wait_for_server(rospy.Duration(0.5)):
            return False, "move_base is not running"
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = Point(loc["x"], loc["y"], 0.0)
        goal.target_pose.pose.orientation = yaw_to_quaternion(loc["yaw"])
        self.active_target = loc["name"]
        self.client.send_goal(goal, done_cb=self._goal_done)
        self.set_status("navigating", loc["name"], "Heading to %s" % loc["name"])
        rospy.loginfo("Navigating to '%s' (x=%.2f y=%.2f)", loc["name"], loc["x"], loc["y"])
        return True, "Navigating to '%s'" % loc["name"]

    def _goal_done(self, state, _result):
        target = self.active_target or ""
        if state == GoalStatus.SUCCEEDED:
            self.set_status("arrived", target, "Arrived at %s" % target)
        elif state in (GoalStatus.PREEMPTED, GoalStatus.RECALLED):
            self.set_status("cancelled", target, "Navigation to %s cancelled" % target)
        else:
            self.set_status("failed", target, "Could not reach %s (move_base state %d)" % (target, state))
        self.active_target = None

    def cancel(self):
        if self.client is None:
            return False, "Navigation is disabled in this session"
        self.client.cancel_all_goals()
        if self.status["state"] == "navigating":
            self.set_status("cancelled", self.active_target or "", "Navigation cancelled")
        self.active_target = None
        return True, "Navigation cancelled"

    def localize_at(self, name):
        loc = self.store.find(name)
        if loc is None:
            return False, "No location named '%s'" % name
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position = Point(loc["x"], loc["y"], 0.0)
        msg.pose.pose.orientation = yaw_to_quaternion(loc["yaw"])
        cov = [0.0] * 36
        cov[0] = 0.25
        cov[7] = 0.25
        cov[35] = 0.0685
        msg.pose.covariance = cov
        self.initialpose_pub.publish(msg)
        return True, "Initial pose set at '%s'" % loc["name"]

    def set_status(self, state, target, message):
        self.status = {"state": state, "target": target, "message": message,
                       "distance": self.status.get("distance", 0.0) if state == "navigating" else 0.0}
        self.publish_status()

    def publish_status(self):
        payload = dict(self.status)
        payload["stamp"] = rospy.Time.now().to_sec()
        self.status_pub.publish(String(data=json.dumps(payload)))

    # ---------------------------------------------------------- publishing --
    def publish_all(self):
        self.publish_list()
        self.publish_markers()
        self.publish_interactive_markers()

    def publish_list(self):
        payload = {"map": self.map_name, "frame_id": self.map_frame, "file": self.locations_file,
                   "navigation_enabled": self.navigation_enabled,
                   "locations": [dict(l) for l in self.store.locations]}
        self.list_pub.publish(String(data=json.dumps(payload)))

    def publish_markers(self):
        array = MarkerArray()
        wipe = Marker()
        wipe.header.frame_id = self.map_frame
        wipe.action = Marker.DELETEALL
        array.markers.append(wipe)
        for i, loc in enumerate(self.store.locations):
            arrow = Marker()
            arrow.header.frame_id = self.map_frame
            arrow.header.stamp = rospy.Time.now()
            arrow.ns = "locations"
            arrow.id = i
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position = Point(loc["x"], loc["y"], 0.05)
            arrow.pose.orientation = yaw_to_quaternion(loc["yaw"])
            arrow.scale = Vector3(0.35, 0.08, 0.08)
            arrow.color = ColorRGBA(0.10, 0.65, 1.0, 0.95)
            array.markers.append(arrow)

            text = Marker()
            text.header = arrow.header
            text.ns = "labels"
            text.id = i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position = Point(loc["x"], loc["y"], 0.45)
            text.pose.orientation.w = 1.0
            text.scale.z = 0.25
            text.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
            text.text = loc["name"]
            array.markers.append(text)
        self.markers_pub.publish(array)

    def publish_interactive_markers(self):
        self.im_server.clear()
        for loc in self.store.locations:
            im = InteractiveMarker()
            im.header.frame_id = self.map_frame
            im.name = loc["name"]
            im.description = loc["name"]
            im.scale = 0.6
            im.pose.position = Point(loc["x"], loc["y"], 0.05)
            im.pose.orientation = yaw_to_quaternion(loc["yaw"])

            visual = Marker()
            visual.type = Marker.CYLINDER
            visual.scale = Vector3(0.30, 0.30, 0.04)
            visual.color = ColorRGBA(0.10, 0.65, 1.0, 0.45)

            button = InteractiveMarkerControl()
            button.interaction_mode = InteractiveMarkerControl.MENU
            button.always_visible = True
            button.markers.append(visual)
            im.controls.append(button)

            move = InteractiveMarkerControl()
            move.orientation = Quaternion(0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5))  # plane normal = z
            move.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
            move.name = "move_rotate"
            im.controls.append(move)

            self.im_server.insert(im, self._im_feedback)
            self.menu.apply(self.im_server, im.name)
        self.im_server.applyChanges()

    # --------------------------------------------------- interactive markers --
    def _im_feedback(self, feedback):
        if feedback.event_type != InteractiveMarkerFeedback.MOUSE_UP:
            return
        loc = self.store.find(feedback.marker_name)
        if loc is None:
            return
        p = feedback.pose.position
        yaw = quaternion_to_yaw(feedback.pose.orientation)
        with self.lock:
            self.store.upsert(loc["name"], p.x, p.y, yaw)
            self.publish_all()
        rospy.loginfo("Moved '%s' to x=%.2f y=%.2f yaw=%.2f", loc["name"], p.x, p.y, yaw)

    def _menu_go(self, feedback):
        ok, msg = self.go_to(feedback.marker_name)
        (rospy.loginfo if ok else rospy.logwarn)(msg)

    def _menu_delete(self, feedback):
        with self.lock:
            self.store.delete(feedback.marker_name)
            self.publish_all()
        rospy.loginfo("Deleted location '%s'", feedback.marker_name)

    def _menu_move_here(self, feedback):
        pose = self.robot_pose()
        if pose is None:
            rospy.logwarn("Robot pose not available")
            return
        with self.lock:
            self.store.upsert(feedback.marker_name, pose[0], pose[1], pose[2])
            self.publish_all()
        rospy.loginfo("Moved '%s' to the robot's current pose", feedback.marker_name)

    def _menu_localize(self, feedback):
        ok, msg = self.localize_at(feedback.marker_name)
        (rospy.loginfo if ok else rospy.logwarn)(msg)


if __name__ == "__main__":
    try:
        LocationManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
