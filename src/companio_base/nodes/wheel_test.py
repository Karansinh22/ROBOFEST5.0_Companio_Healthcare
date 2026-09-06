#!/usr/bin/env python3
"""
wheel_test.py
=============
Interactive wheel tester.  Run the motor driver first, then drive each wheel
individually to confirm wiring, direction and encoder feedback.

  1 / 2   select left / right wheel      a / d   spin selected wheel CCW / CW
  space   stop selected wheel            e / x   speed up / slow down
  f / b   both wheels forward / backward q       quit (stops both wheels)
"""

import sys
import termios
import tty

import rospy
from std_msgs.msg import Float32, UInt32


def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    rospy.init_node("wheel_test", anonymous=True)
    pubs = {"left": rospy.Publisher("lwheel_vtarget", Float32, queue_size=1),
            "right": rospy.Publisher("rwheel_vtarget", Float32, queue_size=1)}
    ticks = {"left": None, "right": None}
    rospy.Subscriber("lwheel_ticks_32bit", UInt32, lambda m: ticks.__setitem__("left", m.data))
    rospy.Subscriber("rwheel_ticks_32bit", UInt32, lambda m: ticks.__setitem__("right", m.data))

    speed = 0.3
    selected = "left"
    print(__doc__)
    rospy.sleep(0.5)

    def status():
        print("\r[%s] speed=%.2f  left_ticks=%s  right_ticks=%s      "
              % (selected.upper(), speed, ticks["left"], ticks["right"]), end="", flush=True)

    try:
        while not rospy.is_shutdown():
            status()
            k = get_key()
            if k == "1":
                selected = "left"
            elif k == "2":
                selected = "right"
            elif k == "a":
                pubs[selected].publish(Float32(data=speed))
            elif k == "d":
                pubs[selected].publish(Float32(data=-speed))
            elif k == " ":
                pubs[selected].publish(Float32(data=0.0))
            elif k == "e":
                speed = min(1.0, speed + 0.05)
            elif k == "x":
                speed = max(0.05, speed - 0.05)
            elif k == "f":
                pubs["left"].publish(Float32(data=speed))
                pubs["right"].publish(Float32(data=-speed))
            elif k == "b":
                pubs["left"].publish(Float32(data=-speed))
                pubs["right"].publish(Float32(data=speed))
            elif k in ("q", "\x03"):
                break
    finally:
        for p in pubs.values():
            p.publish(Float32(data=0.0))
        print("\nWheels stopped.")


if __name__ == "__main__":
    main()
