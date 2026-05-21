#!/usr/bin/env python3
"""Keyboard teleoperation node for RoboRacer simulation.

Publishes AckermannDriveStamped to /ego_racecar/drive.

Controls:
    W / Up    — accelerate
    S / Down  — brake / reverse
    A / Left  — steer left
    D / Right — steer right
    Space     — full stop (speed=0, steering=0)
    Q         — quit

Speed step   : 3.0 m/s forward, 1.5 m/s reverse
Steering     : ±0.35 rad (±20°)
"""

import sys
import time
import termios
import tty
import threading
import queue

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

DRIVE_TOPIC    = '/drive'

SPEED_FWD      = 3.0   # m/s when W is held
SPEED_REV      = 1.5   # m/s when S is held (reverse)
STEER_VAL      = 0.20  # radians when A or D is held (~11°)
PUBLISH_HZ     = 20    # Hz
SPEED_TIMEOUT  = 0.6   # seconds without W before speed zeroes
STEER_TIMEOUT  = 0.5   # seconds without A/D before steer zeroes (~400ms key-repeat delay + margin)
CORNER_FACTOR  = 0.55  # speed multiplier while steering (auto-brakes for bends)

HELP = "\r\n".join([
    "",
    "RoboRacer Keyboard Teleop",
    "─────────────────────────",
    "  W  forward                 S  reverse",
    "  A  steer left              D  steer right",
    "  Space  full stop           Q  quit",
    "─────────────────────────",
    f"  Fwd: {SPEED_FWD} m/s   Rev: {SPEED_REV} m/s   Steer: +/-{STEER_VAL} rad",
    "  Release key to stop/straighten automatically",
    "",
])


_key_queue: queue.Queue = queue.Queue()


def _read_keys():
    """Background thread: reads keypresses and puts them on the queue."""
    while True:
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            # Drain any escape sequence bytes without blocking the main loop
            sys.stdin.read(2)
            continue
        ch = ch.lower()
        _key_queue.put(ch)
        if ch in ('q', '\x03'):
            break


def get_key(timeout: float = 0.05) -> str:
    """Return next keypress from the background reader, or '' on timeout."""
    try:
        return _key_queue.get(timeout=timeout)
    except queue.Empty:
        return ''


class TeleopKey(Node):
    def __init__(self):
        super().__init__('teleop_key')
        self._pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)
        self._speed = 0.0
        self._steer = 0.0
        self._last_speed_time = time.time()
        self._last_steer_time = time.time()
        self._timer = self.create_timer(1.0 / PUBLISH_HZ, self._publish)

    def _publish(self):
        now = time.time()
        if now - self._last_speed_time > SPEED_TIMEOUT:
            self._speed = 0.0
        if now - self._last_steer_time > STEER_TIMEOUT:
            self._steer = 0.0
        # Auto corner-braking: reduce speed while steering is active
        speed = self._speed
        if abs(self._steer) > 0.01 and speed > 0:
            speed = speed * CORNER_FACTOR
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = speed
        msg.drive.steering_angle = self._steer
        self._pub.publish(msg)

    def update(self, key: str) -> bool:
        """Process a key. Returns False if the node should quit."""
        if key in ('q', '\x03'):
            return False
        elif key == 'w':
            self._speed = SPEED_FWD
            self._last_speed_time = time.time()
        elif key == 's':
            self._speed = -SPEED_REV
            self._steer = 0.0                    # zero steer on reverse — prevents spinning out of walls
            self._last_speed_time = time.time()
            self._last_steer_time = time.time()
        elif key == 'a':
            self._steer = STEER_VAL
            self._last_steer_time = time.time()
        elif key == 'd':
            self._steer = -STEER_VAL
            self._last_steer_time = time.time()
        elif key == ' ':
            self._speed = 0.0
            self._steer = 0.0
            self._last_speed_time = time.time()
            self._last_steer_time = time.time()

        if key:
            print(f'\r  speed={self._speed:+.2f} m/s   steer={self._steer:+.3f} rad      ',
                  end='', flush=True)
            sys.stdout.flush()
        return True


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKey()

    # Switch terminal to raw mode so we can read individual keypresses
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        print(HELP)
        print(f'  Publishing to: {DRIVE_TOPIC}\n')

        reader = threading.Thread(target=_read_keys, daemon=True)
        reader.start()

        while rclpy.ok():
            key = get_key(timeout=1.0 / PUBLISH_HZ)
            rclpy.spin_once(node, timeout_sec=0.0)
            if not node.update(key):
                break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print('\nStopped.')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
