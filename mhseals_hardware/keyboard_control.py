"""Deadman keyboard control for an omni boat on the manual lane (``topics.manual``)."""

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
from mhseals_hardware.manual_state import THRUSTERS, ManualState, ThrusterKeys, drain


def manual_panel(active='NEUTRAL', amplitude=0.25, chords=False, thruster=None, velocity=None):
    """Render controls and the currently commanded direction."""
    if velocity is not None:
        speed_line = ('[bold cyan]Speed mode (drift-corrected): the speed controller holds what you ask, '
                      f'{amplitude:.0%} of top speed; V back to direct.[/] Needs drive --with controller and the '
                      'remote in GUIDED.\n' if velocity == 'on' else
                      '[bold red]Speed mode needs the speed tests applied (motion_limits.velocity).[/]\n'
                      if velocity == 'missing' else '[dim]V: speed mode (the controller corrects drift)[/]\n')
    else:
        speed_line = ''
    mode = speed_line + (f'[bold magenta]One-thruster test: {THRUSTERS[thruster]} ({thruster + 1})[/]  W/S drive it alone '
            'forward/reverse; [bold]1-4[/] another, [bold]0[/] normal driving\n' if thruster is not None else
            '[dim]1-4: one-thruster test (FL FR RR RL)[/]\n')
    return Panel(
        '[bold]W/S or ↑/↓[/] forward/reverse    '
        '[bold]A/D[/] port/starboard\n'
        '[bold]←/→[/] rotate CCW/CW             '
        '[bold]Space[/] stop    [bold]X[/] return\n' + mode + '\n'
        f'Command: [yellow]{active}[/]    Speed: {amplitude:.0%} '
        '([bold]+/-[/] adjust by 5%)\n' +
        ('[dim]Chords: hold W+D, add ←/→. Keys stop on release; re-press a held key ~2 s after another is released.[/]'
         if chords else
         '[yellow]Single key: this terminal (or tmux) reports no releases; '
         'newest key drives.[/]') +
        '\n[dim]Space clears all motion. Input timeout clears held keys; '
        'release and press again to re-arm.[/]',
        title='Manual control', border_style='yellow')


def publish_single(publisher, index, effort):
    """One-thruster command [index, effort] (the driver drops it after 0.3 s without a repeat)."""
    from std_msgs.msg import Float32MultiArray
    publisher.publish(Float32MultiArray(data=[float(index), float(effort)]))


def publish_manual(message_publisher, command=(0.0, 0.0, 0.0)):
    """Publish one manual planar command."""
    message = Twist()
    message.linear.x, message.linear.y, message.angular.z = command
    message_publisher.publish(message)


def run_manual(message_publisher, console=None, amplitude=0.25,
               deadman_timeout=0.15, initial_timeout=0.65, status_panel=None, yaw_scale=0.5,
               single_publisher=None, velocity_publisher=None, top_speed=None):
    """Run manual control until X, always finishing with a neutral command. With ``single_publisher``
    keys 1-4 switch to the one-thruster test: W/S's surge effort drives that thruster alone while the
    manual lane sends zero (the driver runs it only then), 0 switches back."""
    console = console or Console()
    state = ManualState(amplitude, deadman_timeout, initial_timeout, yaw_scale)
    picked = None
    try:
        with KeyReader(report_events=True) as raw, Live(
                manual_panel(amplitude=amplitude), console=console,
                refresh_per_second=10) as live:
            keys = ThrusterKeys(raw) if single_publisher is not None or velocity_publisher is not None else raw
            speeding = False
            published = time.monotonic()
            while True:
                command = drain(keys, state, keys.read(timeout=0.05), time.monotonic, published)
                if command is None:
                    return
                thruster = getattr(keys, 'index', None) if single_publisher is not None else None
                wants_speed = bool(getattr(keys, 'velocity', False)) and velocity_publisher is not None
                if speeding and not wants_speed:
                    publish_manual(velocity_publisher)                 # leaving speed mode: ask for zero
                speeding = wants_speed and top_speed is not None
                if speeding:
                    # the keys' fraction of each axis's measured top speed, on the speed controller's input;
                    # nothing on the manual lane, so the mux hands the controller the thrusters
                    asked = (command[0] * top_speed[0], command[1] * top_speed[1],
                             command[2] / (yaw_scale or 1.0) * top_speed[2])
                    publish_manual(velocity_publisher, asked)
                    label = ' '.join(f'{axis} {value:+.2f}{unit}' for axis, value, unit in
                                     zip(('SURGE', 'SWAY', 'YAW'), asked, ('m/s', 'm/s', 'rad/s')) if value)
                    published = time.monotonic()
                    live.update(manual_panel(label or 'HOLD ZERO', state.amplitude, state.chords, None, 'on'))
                    continue
                if picked is not None and thruster != picked:
                    publish_single(single_publisher, picked, 0.0)     # the old one stops at once
                picked = thruster
                if thruster is not None:
                    effort = command[0]
                    publish_manual(message_publisher)
                    publish_single(single_publisher, thruster, effort)
                    label = f'{THRUSTERS[thruster]} alone {effort:+.2f}' if effort else f'{THRUSTERS[thruster]} ready'
                else:
                    label = ' '.join(f'{axis} {value:+.2f}' for axis, value in
                                     zip(('SURGE', 'SWAY', 'YAW'), command) if value)
                    publish_manual(message_publisher, command)
                published = time.monotonic()
                render = status_panel or manual_panel
                if render is manual_panel:
                    mode = None if velocity_publisher is None else (
                        'missing' if wants_speed and top_speed is None else 'off')
                    live.update(render(label or 'NEUTRAL', state.amplitude, state.chords, thruster, mode))
                else:
                    live.update(render(label or 'NEUTRAL', state.amplitude, state.chords))
    finally:
        if picked is not None:
            publish_single(single_publisher, picked, 0.0)
        if velocity_publisher is not None:
            publish_manual(velocity_publisher)
        publish_manual(message_publisher)


def main(args=None):
    """Run standalone keyboard control for a remote thruster node."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', help='shared hardware YAML path')
    parser.add_argument('--speed', type=float, help='initial thrust fraction, 0 to 1')
    parser.add_argument('--dry-run', action='store_true',
                        help='test terminal keys and chords without publishing ROS commands')
    argv = sys.argv if args is None else ['keyboard', *args]
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
                       initial_timeout=config['manual_initial_timeout'],
                       yaw_scale=config['manual_yaw_scale'])
        except (KeyboardInterrupt, EOFError):
            pass
        return
    rclpy.init(args=args)
    node = Node('keyboard_control')
    publisher = node.create_publisher(Twist, config['topics']['manual'], 10)
    speed = node.create_publisher(Twist, config['topics']['auto'], 10) if config['topics'].get('auto') else None
    limits = ((config.get('characterization') or {}).get('motion_limits') or {}).get('velocity') or {}
    top = tuple(limits.get(axis) for axis in ('surge', 'sway', 'yaw'))
    top = top if all(isinstance(v, (int, float)) and v > 0 for v in top) else None
    single = None
    if config['topics'].get('single'):
        from std_msgs.msg import Float32MultiArray
        single = node.create_publisher(Float32MultiArray, config['topics']['single'], 10)
    spinning = True

    def spin():
        while spinning and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    try:
        run_manual(publisher, amplitude=config['manual_amplitude'],
                   deadman_timeout=config['manual_repeat_timeout'],
                   initial_timeout=config['manual_initial_timeout'],
                   yaw_scale=config['manual_yaw_scale'], single_publisher=single,
                   velocity_publisher=speed, top_speed=top)
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
