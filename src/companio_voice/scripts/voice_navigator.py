#!/usr/bin/env python3
"""
voice_navigator.py
==================
Voice command interface for autonomous navigation.  Records short clips from
the on-board microphone (arecord), transcribes them with Google Speech
Recognition and turns the text into navigation or motion commands.

Spoken commands
  "go to <room>", "take me to <room>", "navigate to <room>"   -> /locations/go_to
  "go home"                                                    -> /locations/go_to Home
  "save this as <room>", "remember this as <room>"            -> /locations/save
  "cancel", "stop", "halt"                                     -> cancel goal and stop motors
  "forward", "back", "left", "right"                           -> short manual motion
  "where are you"                                              -> reads the robot pose aloud in the log
  "over", "exit", "quit"                                       -> stop listening

Requires the location manager (navigation.launch) to be running.
"""

import difflib
import json
import os
import re
import subprocess

import rospy
import speech_recognition as sr
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger

from companio_msgs.srv import LocationName, SaveLocation


class VoiceNavigator:
    GO_PATTERNS = [r"(?:go|navigate|drive|move|take me|head) (?:to |towards |into )?(?:the )?(.+)"]
    SAVE_PATTERNS = [r"(?:save|remember|store) (?:this|here|location|position)? ?(?:as|called|named)? ?(.+)"]

    def __init__(self):
        rospy.init_node("voice_navigator", anonymous=True)
        self.audio_device = rospy.get_param("~audio_device", "plughw:2,0")
        self.sample_rate = int(rospy.get_param("~sample_rate", 16000))
        self.record_seconds = int(rospy.get_param("~record_seconds", 3))
        self.linear_speed = float(rospy.get_param("~linear_speed", 0.25))
        self.angular_speed = float(rospy.get_param("~angular_speed", 0.8))
        self.audio_file = "/tmp/companio_voice_command.wav"

        self.recognizer = sr.Recognizer()
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.locations = []
        self.pose = None
        rospy.Subscriber("/locations/list", String, self._on_list)
        rospy.Subscriber("/robot_pose", PoseStamped, self._on_pose)
        self.go_srv = rospy.ServiceProxy("/locations/go_to", LocationName)
        self.save_srv = rospy.ServiceProxy("/locations/save", SaveLocation)
        self.cancel_srv = rospy.ServiceProxy("/locations/cancel", Trigger)
        rospy.loginfo("Voice navigator ready (mic %s). Say 'go to <room>', 'stop', 'go home' ...",
                      self.audio_device)

    def _on_list(self, msg):
        self.locations = [l["name"] for l in json.loads(msg.data).get("locations", [])]

    def _on_pose(self, msg):
        self.pose = msg

    # ------------------------------------------------------------- audio ----
    def listen(self):
        rospy.loginfo("Listening (%ds) ...", self.record_seconds)
        result = subprocess.run(["arecord", "-D", self.audio_device, "-f", "S16_LE",
                                 "-r", str(self.sample_rate), "-c", "1",
                                 "-d", str(self.record_seconds), self.audio_file],
                                capture_output=True, text=True)
        if result.returncode != 0 or not os.path.exists(self.audio_file):
            rospy.logerr("Recording failed: %s", result.stderr.strip())
            return None
        try:
            with sr.AudioFile(self.audio_file) as source:
                audio = self.recognizer.record(source)
            text = self.recognizer.recognize_google(audio).lower().strip()
            rospy.loginfo("Heard: '%s'", text)
            return text
        except sr.UnknownValueError:
            rospy.loginfo("(nothing understood)")
        except sr.RequestError as exc:
            rospy.logerr("Speech service error: %s", exc)
        return None

    # ---------------------------------------------------------- commands ----
    def resolve(self, spoken):
        spoken = spoken.strip()
        if not self.locations:
            return spoken
        for name in self.locations:
            if name.lower() == spoken.lower():
                return name
        match = difflib.get_close_matches(spoken, self.locations, n=1, cutoff=0.4)
        return match[0] if match else spoken

    def drive(self, linear, angular):
        t = Twist()
        t.linear.x = linear
        t.angular.z = angular
        self.cmd_pub.publish(t)

    def handle(self, text):
        if any(w in text for w in ("over", "exit", "quit")):
            self.drive(0.0, 0.0)
            return False
        if any(w in text for w in ("stop", "cancel", "halt", "wait")):
            try:
                self.cancel_srv()
            except rospy.ServiceException:
                pass
            self.drive(0.0, 0.0)
            rospy.loginfo("Stopped")
            return True
        if "home" in text and ("go" in text or "return" in text or "back" in text):
            return self._go("Home")
        for pat in self.SAVE_PATTERNS:
            m = re.search(pat, text)
            if m and any(k in text for k in ("save", "remember", "store")):
                return self._save(m.group(1).strip())
        for pat in self.GO_PATTERNS:
            m = re.search(pat, text)
            if m:
                return self._go(self.resolve(m.group(1)))
        if "where" in text and self.pose is not None:
            p = self.pose.pose.position
            rospy.loginfo("I am at x=%.2f y=%.2f on the map", p.x, p.y)
            return True
        if "forward" in text or "ahead" in text:
            self.drive(self.linear_speed, 0.0)
        elif "back" in text or "reverse" in text:
            self.drive(-self.linear_speed, 0.0)
        elif "left" in text:
            self.drive(0.0, self.angular_speed)
        elif "right" in text:
            self.drive(0.0, -self.angular_speed)
        else:
            rospy.logwarn("Unknown command: '%s'", text)
        return True

    def _go(self, name):
        try:
            r = self.go_srv(name=name)
            (rospy.loginfo if r.success else rospy.logwarn)(r.message)
        except rospy.ServiceException as exc:
            rospy.logerr("go_to failed: %s (is navigation.launch running?)", exc)
        return True

    def _save(self, name):
        try:
            r = self.save_srv(name=name, use_robot_pose=True)
            (rospy.loginfo if r.success else rospy.logwarn)(r.message)
        except rospy.ServiceException as exc:
            rospy.logerr("save failed: %s", exc)
        return True

    def run(self):
        try:
            while not rospy.is_shutdown():
                text = self.listen()
                if text and not self.handle(text):
                    break
        finally:
            self.drive(0.0, 0.0)
            if os.path.exists(self.audio_file):
                os.remove(self.audio_file)
            rospy.loginfo("Voice navigator stopped")


if __name__ == "__main__":
    try:
        VoiceNavigator().run()
    except rospy.ROSInterruptException:
        pass
