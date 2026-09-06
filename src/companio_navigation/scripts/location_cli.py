#!/usr/bin/env python3
"""
location_cli.py
===============
Terminal client for the location manager.  Works over SSH, no display needed.

Interactive mode:
    rosrun companio_navigation location_cli.py

One-shot mode:
    rosrun companio_navigation location_cli.py save "Ward A"
    rosrun companio_navigation location_cli.py go "Ward A"
    rosrun companio_navigation location_cli.py list

Commands
    save [name]          save the robot's current pose (auto-named if omitted)
    list                 list saved locations
    go <name>            navigate to a location (fuzzy name matching)
    home                 navigate to "Home"
    delete <name>        delete a location
    rename <old> <new>   rename a location
    localize <name>      tell AMCL the robot is standing at <name>
    cancel               cancel the active navigation goal
    pose                 print the robot pose in the map frame
    status               print the navigation status
    help / quit
"""

import difflib
import json
import shlex
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger

from companio_msgs.srv import LocationName, RenameLocation, SaveLocation


class LocationClient:
    def __init__(self):
        self.locations = []
        self.status = {}
        self.pose = None
        rospy.Subscriber("/locations/list", String, self._on_list)
        rospy.Subscriber("/navigation/status", String, self._on_status)
        rospy.Subscriber("/robot_pose", PoseStamped, self._on_pose)
        self._save = rospy.ServiceProxy("/locations/save", SaveLocation)
        self._delete = rospy.ServiceProxy("/locations/delete", LocationName)
        self._rename = rospy.ServiceProxy("/locations/rename", RenameLocation)
        self._go = rospy.ServiceProxy("/locations/go_to", LocationName)
        self._localize = rospy.ServiceProxy("/locations/localize_at", LocationName)
        self._cancel = rospy.ServiceProxy("/locations/cancel", Trigger)

    def _on_list(self, msg):
        self.locations = json.loads(msg.data).get("locations", [])

    def _on_status(self, msg):
        self.status = json.loads(msg.data)

    def _on_pose(self, msg):
        self.pose = msg

    def wait(self, timeout=5.0):
        try:
            rospy.wait_for_service("/locations/save", timeout)
        except rospy.ROSException:
            print("location_manager is not running. Start mapping.launch or navigation.launch first.")
            sys.exit(1)

    def resolve(self, name):
        names = [l["name"] for l in self.locations]
        if not names:
            return name
        exact = [n for n in names if n.lower() == name.lower()]
        if exact:
            return exact[0]
        match = difflib.get_close_matches(name, names, n=1, cutoff=0.5)
        if match:
            print("  (using closest match '%s')" % match[0])
            return match[0]
        return name

    # -- commands ---------------------------------------------------------------
    def cmd_list(self):
        if not self.locations:
            print("  no locations saved yet")
        for l in self.locations:
            print("  %-20s x=%7.2f  y=%7.2f  yaw=%6.2f" % (l["name"], l["x"], l["y"], l["yaw"]))

    def cmd_save(self, name=""):
        r = self._save(name=name, use_robot_pose=True)
        print("  " + r.message)

    def cmd_go(self, name):
        r = self._go(name=self.resolve(name))
        print("  " + r.message)

    def cmd_delete(self, name):
        r = self._delete(name=self.resolve(name))
        print("  " + r.message)

    def cmd_rename(self, old, new):
        r = self._rename(old_name=self.resolve(old), new_name=new)
        print("  " + r.message)

    def cmd_localize(self, name):
        r = self._localize(name=self.resolve(name))
        print("  " + r.message)

    def cmd_cancel(self):
        r = self._cancel()
        print("  " + r.message)

    def cmd_pose(self):
        if self.pose is None:
            print("  robot pose not available yet")
            return
        p = self.pose.pose
        import math
        yaw = math.atan2(2 * (p.orientation.w * p.orientation.z), 1 - 2 * p.orientation.z ** 2)
        print("  x=%.2f y=%.2f yaw=%.2f (%s)" % (p.position.x, p.position.y, yaw, self.pose.header.frame_id))

    def cmd_status(self):
        if not self.status:
            print("  no status yet")
            return
        print("  state=%s target=%s distance=%s  %s" % (
            self.status.get("state"), self.status.get("target") or "-",
            self.status.get("distance"), self.status.get("message", "")))

    def run(self, argv):
        if not argv:
            return True
        cmd, args = argv[0].lower(), argv[1:]
        try:
            if cmd in ("list", "ls"):
                self.cmd_list()
            elif cmd == "save":
                self.cmd_save(" ".join(args))
            elif cmd == "go":
                if not args:
                    print("  usage: go <name>")
                else:
                    self.cmd_go(" ".join(args))
            elif cmd == "home":
                self.cmd_go("Home")
            elif cmd in ("delete", "del", "rm"):
                self.cmd_delete(" ".join(args))
            elif cmd == "rename":
                if len(args) != 2:
                    print('  usage: rename "old name" "new name"')
                else:
                    self.cmd_rename(args[0], args[1])
            elif cmd == "localize":
                self.cmd_localize(" ".join(args))
            elif cmd in ("cancel", "stop"):
                self.cmd_cancel()
            elif cmd == "pose":
                self.cmd_pose()
            elif cmd == "status":
                self.cmd_status()
            elif cmd in ("help", "?"):
                print(__doc__)
            elif cmd in ("quit", "exit", "q"):
                return False
            else:
                print("  unknown command '%s' (type help)" % cmd)
        except rospy.ServiceException as exc:
            print("  service call failed: %s" % exc)
        return True


def main():
    rospy.init_node("location_cli", anonymous=True)
    client = LocationClient()
    client.wait()
    rospy.sleep(0.3)
    args = rospy.myargv(argv=sys.argv)[1:]
    if args:
        client.run(args)
        return
    print("Companio location console - type 'help' for commands, 'quit' to exit.")
    client.cmd_list()
    while not rospy.is_shutdown():
        try:
            line = input("companio> ")
        except (EOFError, KeyboardInterrupt):
            print()
            break
        try:
            argv = shlex.split(line)
        except ValueError as exc:
            print("  parse error: %s" % exc)
            continue
        if not client.run(argv):
            break


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
