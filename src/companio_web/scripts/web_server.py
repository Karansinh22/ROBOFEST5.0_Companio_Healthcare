#!/usr/bin/env python3
"""
web_server.py
=============
Serves the Companio dashboard (www/) over plain HTTP so it can be opened on the
robot's touchscreen, a phone or a laptop on the same network.  The page talks
to ROS through rosbridge (websocket, default port 9090).
"""

import functools
import os
import socket
import threading
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

import rospkg
import rospy


class QuietHandler(SimpleHTTPRequestHandler):
    def log_message(self, fmt, *args):  # keep the ROS console readable
        rospy.logdebug("http: " + fmt, *args)

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()


def local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return "127.0.0.1"


def main():
    rospy.init_node("web_server")
    port = int(rospy.get_param("~port", 8000))
    default_root = os.path.join(rospkg.RosPack().get_path("companio_web"), "www")
    root = rospy.get_param("~root", default_root)

    handler = functools.partial(QuietHandler, directory=root)
    server = ThreadingHTTPServer(("0.0.0.0", port), handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    rospy.loginfo("Companio dashboard: http://%s:%d/  (serving %s)", local_ip(), port, root)
    rospy.on_shutdown(server.shutdown)
    rospy.spin()


if __name__ == "__main__":
    main()
