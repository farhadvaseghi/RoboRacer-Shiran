#!/usr/bin/env python3
"""Keyboard teleoperation node for RoboRacer simulation.

Publishes AckermannDriveStamped to /drive.

Controls:
    W / Up    - accelerate
    S / Down  - brake / reverse
    A / Left  - steer left
    D / Right - steer right
    Space     - full stop (speed=0, steering=0)
    Q         - quit
"""

import os
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from Xlib import X, XK, display
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node

DRIVE_TOPIC = '/drive'

SPEED_FWD = 3.5
SPEED_REV = 3.0
STEER_VAL = 0.18
PUBLISH_HZ = 20
CORNER_FACTOR = 0.82
FORWARD_HOLD_GRACE = 0.60
REVERSE_HOLD_GRACE = 0.35
STEER_HOLD_GRACE = 0.25
STDIN_REPEAT_GRACE = 0.18

HELP = "\r\n".join([
    "",
    "RoboRacer Keyboard Teleop",
    "-------------------------",
    "  W  forward                 S  reverse",
    "  A  steer left              D  steer right",
    "  Space  full stop           Q  quit",
    "-------------------------",
    f"  Fwd: {SPEED_FWD} m/s   Rev: {SPEED_REV} m/s   Steer: +/-{STEER_VAL} rad",
    "  Hold combinations like W+A, W+D, S+A, or S+D for combined motion",
    "",
])


class XKeyboardState:
    """Read current keyboard state with event-backed repeat filtering."""

    def __init__(self) -> None:
        if not os.environ.get('DISPLAY'):
            raise RuntimeError('DISPLAY is not set')
        self._display = display.Display()
        self._event_display = display.Display()
        self._root = self._event_display.screen().root
        self._root.change_attributes(event_mask=X.KeyPressMask | X.KeyReleaseMask)
        self._keycodes = {
            'w': self._keysym_code('w'),
            'a': self._keysym_code('a'),
            's': self._keysym_code('s'),
            'd': self._keysym_code('d'),
            'q': self._keysym_code('q'),
            'space': self._keysym_code('space'),
            'up': self._keysym_code('Up'),
            'down': self._keysym_code('Down'),
            'left': self._keysym_code('Left'),
            'right': self._keysym_code('Right'),
        }
        self._pressed_events = {name: False for name in self._keycodes}
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._event_loop, daemon=True)
        self._thread.start()

    def _keysym_code(self, name: str) -> int:
        keysym = XK.string_to_keysym(name)
        if keysym == 0:
            raise RuntimeError(f'Unable to resolve keysym for {name}')
        return self._display.keysym_to_keycode(keysym)

    @staticmethod
    def _pressed(bitmap: bytes, keycode: int) -> bool:
        return bool(bitmap[keycode // 8] & (1 << (keycode % 8)))

    def _event_loop(self) -> None:
        code_to_name = {code: name for name, code in self._keycodes.items()}
        while True:
            event = self._event_display.next_event()
            key_name = code_to_name.get(getattr(event, 'detail', None))
            if key_name is None:
                continue

            if event.type == X.KeyPress:
                with self._lock:
                    self._pressed_events[key_name] = True
                continue

            if event.type != X.KeyRelease:
                continue

            time.sleep(0.008)
            repeated = False
            while self._event_display.pending_events():
                next_event = self._event_display.next_event()
                next_name = code_to_name.get(getattr(next_event, 'detail', None))
                if next_name is None:
                    continue
                if (
                    next_event.type == X.KeyPress
                    and next_name == key_name
                ):
                    with self._lock:
                        self._pressed_events[key_name] = True
                    repeated = True
                    break
                if next_event.type == X.KeyPress:
                    with self._lock:
                        self._pressed_events[next_name] = True
                elif next_event.type == X.KeyRelease:
                    with self._lock:
                        self._pressed_events[next_name] = False

            if not repeated:
                with self._lock:
                    self._pressed_events[key_name] = False

    def snapshot(self) -> dict[str, bool]:
        bitmap = self._display.query_keymap()
        with self._lock:
            return {
                name: self._pressed(bitmap, code) or self._pressed_events[name]
                for name, code in self._keycodes.items()
            }


class TerminalRepeatState:
    """Track recent key repeats coming from the controlling terminal."""

    def __init__(self) -> None:
        self._enabled = sys.stdin.isatty()
        self._timestamps = {
            'w': 0.0,
            'a': 0.0,
            's': 0.0,
            'd': 0.0,
            'q': 0.0,
            'space': 0.0,
            'up': 0.0,
            'down': 0.0,
            'left': 0.0,
            'right': 0.0,
        }
        self._lock = threading.Lock()
        self._stop = False
        self._fd = None
        self._old_termios = None
        if not self._enabled:
            return
        self._fd = sys.stdin.fileno()
        self._old_termios = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

    def _mark(self, key_name: str) -> None:
        with self._lock:
            self._timestamps[key_name] = time.monotonic()

    def _reader_loop(self) -> None:
        while not self._stop:
            ready, _, _ = select.select([self._fd], [], [], 0.05)
            if not ready:
                continue
            try:
                chunk = os.read(self._fd, 32)
            except OSError:
                continue
            if not chunk:
                continue
            self._consume_bytes(chunk)

    def _consume_bytes(self, chunk: bytes) -> None:
        index = 0
        while index < len(chunk):
            byte = chunk[index]
            if byte == 0x1B and index + 2 < len(chunk) and chunk[index + 1] == 0x5B:
                arrow = chunk[index + 2]
                if arrow == 0x41:
                    self._mark('up')
                elif arrow == 0x42:
                    self._mark('down')
                elif arrow == 0x43:
                    self._mark('right')
                elif arrow == 0x44:
                    self._mark('left')
                index += 3
                continue

            char = chr(byte).lower()
            if char in ('w', 'a', 's', 'd', 'q', ' '):
                self._mark('space' if char == ' ' else char)
            index += 1

    def snapshot(self) -> dict[str, bool]:
        if not self._enabled:
            return {name: False for name in self._timestamps}
        now = time.monotonic()
        with self._lock:
            return {
                name: (now - timestamp) <= STDIN_REPEAT_GRACE
                for name, timestamp in self._timestamps.items()
            }

    def close(self) -> None:
        if not self._enabled:
            return
        self._stop = True
        if self._old_termios is not None and self._fd is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_termios)


class TeleopKey(Node):
    def __init__(self):
        super().__init__('teleop_key')
        self._pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)
        self._keyboard = XKeyboardState()
        self._terminal = TerminalRepeatState()
        self._quit_requested = False
        self._last_status = None
        now = time.monotonic()
        self._last_seen = {
            'forward': now - FORWARD_HOLD_GRACE * 2.0,
            'reverse': now - REVERSE_HOLD_GRACE * 2.0,
            'left': now - STEER_HOLD_GRACE * 2.0,
            'right': now - STEER_HOLD_GRACE * 2.0,
        }
        self._timer = self.create_timer(1.0 / PUBLISH_HZ, self._publish)

    def _held(self, pressed: bool, key_name: str, now: float, grace: float) -> bool:
        if pressed:
            self._last_seen[key_name] = now
            return True
        return (now - self._last_seen[key_name]) <= grace

    def _publish(self):
        state = self._keyboard.snapshot()
        terminal_state = self._terminal.snapshot()
        state = {
            name: state.get(name, False) or terminal_state.get(name, False)
            for name in state
        }
        now = time.monotonic()
        if state['q']:
            self._quit_requested = True

        stop = state['space']
        forward = self._held(state['w'] or state['up'], 'forward', now, FORWARD_HOLD_GRACE)
        reverse = self._held(state['s'] or state['down'], 'reverse', now, REVERSE_HOLD_GRACE)
        left = self._held(state['a'] or state['left'], 'left', now, STEER_HOLD_GRACE)
        right = self._held(state['d'] or state['right'], 'right', now, STEER_HOLD_GRACE)

        if stop:
            speed = 0.0
            steer = 0.0
        else:
            if forward and not reverse:
                speed = SPEED_FWD
            elif reverse and not forward:
                speed = -SPEED_REV
            else:
                speed = 0.0

            if left and not right:
                steer = STEER_VAL
            elif right and not left:
                steer = -STEER_VAL
            else:
                steer = 0.0

        if abs(steer) > 0.01 and speed > 0:
            speed = speed * CORNER_FACTOR

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = speed
        msg.drive.steering_angle = steer
        self._pub.publish(msg)

        status = (round(speed, 2), round(steer, 3))
        if status != self._last_status:
            self._last_status = status
            print(
                f'\r  speed={speed:+.2f} m/s   steer={steer:+.3f} rad      ',
                end='',
                flush=True,
            )

    def should_quit(self) -> bool:
        return self._quit_requested


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKey()

    try:
        print(HELP)
        print(f'  Publishing to: {DRIVE_TOPIC}\n')
        try:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=1.0 / PUBLISH_HZ)
                if node.should_quit():
                    break
        except KeyboardInterrupt:
            pass
    finally:
        try:
            stop_msg = AckermannDriveStamped()
            stop_msg.header.stamp = node.get_clock().now().to_msg()
            stop_msg.drive.speed = 0.0
            stop_msg.drive.steering_angle = 0.0
            node._pub.publish(stop_msg)
        except Exception:
            pass
        print('\nStopped.')
        node._terminal.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
