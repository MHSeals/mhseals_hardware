"""Deadman keyboard control for an omni boat through ``cmd_vel``."""

import argparse
import sys
import threading
import time
from types import SimpleNamespace

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from rich.console import Console
from rich.live import Live
from rich.panel import Panel

from mhseals_hardware.keyboard import KeyReader
from mhseals_hardware.configuration import load_config
from mhseals_hardware.manual_state import ManualState


def manual_panel(active='NEUTRAL', amplitude=0.25, event_mode=False):
    """Render controls and the currently commanded direction."""
    return Panel(
        '[bold]W/S or ↑/↓[/] forward/reverse    '
        '[bold]A/D[/] port/starboard\n'
        '[bold]←/→[/] rotate CCW/CW             '
        '[bold]Space[/] stop    [bold]X[/] return\n\n'
        f'Command: [yellow]{active}[/]    Speed: {amplitude:.0%} '
        '([bold]+/-[/] adjust by 5%)\n' +
        ('[dim]Multi-key mode: W+D diagonal; arrows add yaw. Release stops each key.[/]'
         if event_mode else
         '[yellow]Single-key fallback: terminal has not sent key events. '
         'Use a Kitty-protocol terminal for held-key combinations.[/]') +
        '\n[dim]Space clears all motion. Input timeout clears held keys; '
        'release and press again to re-arm.[/]',
        title='Manual control', border_style='yellow')


def publish_manual(message_publisher, command=(0.0, 0.0, 0.0)):
    """Publish one manual planar command."""
    message = Twist()
    message.linear.x, message.linear.y, message.angular.z = command
    message_publisher.publish(message)


def run_manual(message_publisher, console=None, amplitude=0.25,
               deadman_timeout=0.15, initial_timeout=0.65, status_panel=None):
    """Run manual control until X, always finishing with a neutral command."""
    console = console or Console()
    state = ManualState(amplitude, deadman_timeout, initial_timeout)
    try:
        with KeyReader(report_events=True) as keys, Live(
                manual_panel(amplitude=amplitude), console=console,
                refresh_per_second=10) as live:
            while True:
                key = keys.read(timeout=0.05)
                # Consume already queued events before publishing one coherent
                # command; never publish an intermediate neutral between events.
                for index in range(64):
                    if key in ('x', 'm', 'escape', 'press:x', 'press:m', 'press:escape'):
                        return
                    command = state.update(key, time.monotonic())
                    if key in ('space', 'press:space', 'focus-out'):
                        break  # Stop must not be hidden by queued input.
                    if index == 63:
                        break
                    key = keys.read(timeout=0)
                    if key is None:
                        break
                label = ' '.join(f'{axis} {value:+.2f}' for axis, value in
                                 zip(('SURGE', 'SWAY', 'YAW'), command) if value)
                publish_manual(message_publisher, command)
                render = status_panel or manual_panel
                live.update(render(label or 'NEUTRAL', state.amplitude, state.event_mode))
    finally:
        publish_manual(message_publisher)


def main(args=None):
    """Run standalone keyboard control for a remote thruster node."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', help='shared hardware YAML path')
    parser.add_argument('--speed', type=float, help='initial thrust fraction, 0 to 1')
    parser.add_argument('--dry-run', action='store_true',
                        help='test terminal keys and chords without publishing ROS commands')
    argv = sys.argv if args is None else ['keyboard_control', *args]
    options = parser.parse_args(remove_ros_args(args=argv)[1:])
    if options.speed is not None and not 0 <= options.speed <= 1:
        parser.error('--speed must be between 0 and 1')
    config = load_config(options.config)
    if options.speed is not None:
        config['manual_amplitude'] = options.speed
    if options.dry_run:
        console = Console()
        console.print('[bold green]DRY RUN: no ROS commands or hardware outputs[/]')
        try:
            run_manual(SimpleNamespace(publish=lambda message: None), console,
                       amplitude=config['manual_amplitude'],
                       deadman_timeout=config['manual_repeat_timeout'],
                       initial_timeout=config['manual_initial_timeout'])
        except (KeyboardInterrupt, EOFError):
            pass
        return
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
        run_manual(publisher, amplitude=config['manual_amplitude'],
                   deadman_timeout=config['manual_repeat_timeout'],
                   initial_timeout=config['manual_initial_timeout'])
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        publish_manual(publisher)
        spinning = False
        thread.join(timeout=1)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
