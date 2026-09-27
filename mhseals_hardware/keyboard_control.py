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
from mhseals_hardware.configuration import load_config
from mhseals_hardware.manual_state import ManualState


def manual_panel(active='NEUTRAL', amplitude=0.25):
    """Render controls and the currently commanded direction."""
    return Panel(
        '[bold]W/S or ↑/↓[/] forward/reverse    '
        '[bold]A/D[/] port/starboard\n'
        '[bold]←/→[/] rotate CCW/CW             '
        '[bold]Space[/] stop    [bold]X[/] return\n\n'
        f'Command: [yellow]{active}[/]    Speed: {amplitude:.0%} '
        '([bold]+/-[/] adjust by 5%)\n'
        '[dim]Release-aware terminals stop immediately. Otherwise: initial '
        'repeat grace, then short repeat timeout. Space always stops.[/]',
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
               deadman_timeout=0.15, initial_timeout=0.65):
    """Run manual control until X, always finishing with a neutral command."""
    console = console or Console()
    state = ManualState(amplitude, deadman_timeout, initial_timeout)
    try:
        with KeyReader(report_events=True) as keys, Live(
                manual_panel(amplitude=amplitude), console=console,
                refresh_per_second=10) as live:
            while True:
                key = keys.read(timeout=0.05)
                now = time.monotonic()
                if key in ('x', 'escape'):
                    return
                axis, value = state.update(key, now)
                label = 'NEUTRAL' if axis is None else f'{axis.upper()} {value:+.2f}'
                publish_manual(message_publisher, axis, value)
                live.update(manual_panel(label, state.amplitude))
    finally:
        publish_manual(message_publisher)


def main(args=None):
    """Run standalone keyboard control for a remote thruster node."""
    rclpy.init(args=args)
    node = Node('keyboard_control')
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    spinning = True

    def spin():
        while spinning and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    try:
        config = load_config()
        run_manual(publisher, amplitude=config['manual_amplitude'],
                   deadman_timeout=config['manual_repeat_timeout'],
                   initial_timeout=config['manual_initial_timeout'])
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
