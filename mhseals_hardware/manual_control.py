"""Deadman keyboard control for an omni boat through ``cmd_vel``."""

import threading
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rich.console import Console
from rich.live import Live
from rich.panel import Panel

from mhseals_hardware.keyboard import KeyReader


MANUAL_KEYS = {
    'w': ('surge', 1.0), 'up': ('surge', 1.0),
    's': ('surge', -1.0), 'down': ('surge', -1.0),
    'a': ('sway', 1.0), 'd': ('sway', -1.0),
    'left': ('yaw', 1.0), 'right': ('yaw', -1.0),
}


def manual_panel(active='NEUTRAL'):
    """Render controls and the currently commanded direction."""
    return Panel(
        '[bold]W/S or ↑/↓[/] forward/reverse    '
        '[bold]A/D[/] port/starboard\n'
        '[bold]←/→[/] rotate CCW/CW             '
        '[bold]Space[/] stop    [bold]X[/] return\n\n'
        f'Command: [yellow]{active}[/]\n'
        '[dim]Deadman: command stops unless keys continue arriving.[/]',
        title='Manual control', border_style='yellow')


def publish_manual(message_publisher, axis=None, value=0.0):
    """Publish one manual planar command."""
    message = Twist()
    if axis == 'surge':
        message.linear.x = value
    elif axis == 'sway':
        message.linear.y = value
    elif axis == 'yaw':
        message.angular.z = value
    message_publisher.publish(message)


def run_manual(message_publisher, console=None, amplitude=0.25,
               deadman_timeout=0.35):
    """Run manual control until X, always finishing with a neutral command."""
    console = console or Console()
    axis = None
    value = 0.0
    label = 'NEUTRAL'
    deadline = 0.0
    try:
        with KeyReader() as keys, Live(
                manual_panel(), console=console,
                refresh_per_second=10) as live:
            while True:
                key = keys.read(timeout=0.05)
                now = time.monotonic()
                if key in ('x', 'escape'):
                    return
                if key == 'space':
                    axis, value, label, deadline = None, 0.0, 'NEUTRAL', 0.0
                elif key in MANUAL_KEYS:
                    axis, direction = MANUAL_KEYS[key]
                    value = direction * amplitude
                    label = f'{axis.upper()} {"+" if direction > 0 else "-"}'
                    deadline = now + deadman_timeout
                elif deadline and now >= deadline:
                    axis, value, label, deadline = None, 0.0, 'NEUTRAL', 0.0
                publish_manual(message_publisher, axis, value)
                live.update(manual_panel(label))
    finally:
        publish_manual(message_publisher)


def main(args=None):
    """Run standalone keyboard control for a remote thruster node."""
    rclpy.init(args=args)
    node = Node('boat_manual_control')
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    spinning = True

    def spin():
        while spinning and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    try:
        run_manual(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publish_manual(publisher)
        spinning = False
        thread.join(timeout=1)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
