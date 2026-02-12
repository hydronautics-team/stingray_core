#!/usr/bin/env python3
"""
Keyboard Publisher Node for ROS2
Publishes geometry_msgs/Twist to a topic based on keyboard input.
Safe to launch via ros2 launch: disables keyboard input if stdin is not a tty.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import queue
import time
import sys
import os
import select

# Try importing termios/tty only if available
try:
    import tty
    import termios
except ImportError:
    tty = None
    termios = None

KEY_MAP = {
    # linear
    'w': ("surge", +1), 's': ("surge", -1),
    'a': ("sway",  -1), 'd': ("sway",  +1),
    'r': ("depth", -1), 'f': ("depth", +1),
    # angular
    'i': ("roll",  +1), 'k': ("roll",  -1),
    'j': ("pitch", +1), 'l': ("pitch", -1),
    'u': ("yaw",   +1), 'o': ("yaw",   -1),
}

HELP_TEXT = """
Keyboard control:
 w/s  -> surge (forward/back)
 a/d  -> sway (left/right)
 r/f  -> depth (up/down)
 i/k  -> roll
 j/l  -> pitch
 u/o  -> yaw
 space -> zero all axes
 p     -> toggle publish
 + / - -> increase/decrease step
 h     -> show this help
 q     -> quit
"""

class KeyboardPublisher(Node):
    def __init__(self, topic="topic_control_data", hz=20.0, step=0.1):
        super().__init__('keyboard_publisher')
        self.pub = self.create_publisher(Twist, topic, 10)
        self.hz = float(hz)
        self.step = float(step)
        self.values = {axis:0.0 for axis in ["surge","sway","depth","roll","pitch","yaw"]}
        self.publish_enabled = False
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.get_logger().info(f"KeyboardPublisher started. Publishing to '{topic}' at {hz} Hz")

        # start timer
        period = 1.0 / max(1e-3, self.hz)
        self.timer = self.create_timer(period, self._on_timer)

        # start keyboard thread only if stdin is a tty
        if sys.stdin and os.isatty(sys.stdin.fileno()) and tty and termios:
            self.kb_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
            self.kb_thread.start()
        else:
            self.get_logger().warning("stdin is not a tty, keyboard input disabled")

    def _on_timer(self):
        if not self.publish_enabled:
            return
        msg = Twist()
        with self.lock:
            msg.linear.x = self.values["surge"]
            msg.linear.y = self.values["sway"]
            msg.linear.z = self.values["depth"]
            msg.angular.x = self.values["roll"]
            msg.angular.y = self.values["pitch"]
            msg.angular.z = self.values["yaw"]
        self.pub.publish(msg)

    def keyboard_loop(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setraw(fd)
        try:
            while not self.stop_event.is_set():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    ch = sys.stdin.read(1)
                    self.process_key(ch)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def process_key(self, ch):
        ch = ch.lower()
        if ch in KEY_MAP:
            axis, mult = KEY_MAP[ch]
            with self.lock:
                self.values[axis] += self.step * mult
            self.get_logger().info(self.status())
        elif ch == ' ':
            with self.lock:
                for k in self.values:
                    self.values[k] = 0.0
            self.get_logger().info("ZERO -> " + self.status())
        elif ch == 'p':
            self.publish_enabled = not self.publish_enabled
            self.get_logger().info(f"Publishing {'ENABLED' if self.publish_enabled else 'DISABLED'}")
        elif ch == '+':
            self.step *= 2
            self.get_logger().info(f"Step set to {self.step}")
        elif ch == '-':
            self.step /= 2
            self.get_logger().info(f"Step set to {self.step}")
        elif ch == 'h':
            self.get_logger().info(HELP_TEXT)
        elif ch == 'q' or ord(ch)==3:  # Ctrl-C
            self.get_logger().info("Exit requested")
            self.stop_event.set()
            rclpy.shutdown()

    def status(self):
        with self.lock:
            vals = ", ".join(f"{k}={v:.3f}" for k,v in self.values.items())
            return f"publish={'ON' if self.publish_enabled else 'OFF'} | step={self.step:.3f} | {vals}"


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_event.set()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
