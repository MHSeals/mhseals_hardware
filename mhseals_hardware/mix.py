"""Print every mixer stage for one effort command, as the driver computes it.

    ros2 run mhseals_hardware mix SURGE SWAY YAW [--config FILE]
    ros2 run mhseals_hardware mix --live [--config FILE]

Layers as in docs/CONTROL.md: L3 decoupling (C or C- per axis), L2 geometry mix (M rows, thrust
sum, desaturation), L1 thruster law (e|e|, reverse ratio), L0 PWM (reversed, pulse, channel map).
--live follows topics.applied (the mux output the driver mixes) and reprints on each change.
The config is loaded like the driver's (default.yaml < vessel thrusters: < FILE); a value set
live with ros2 param set on the driver is not seen here until it is applied to the file.
"""

import argparse
import sys

from mhseals_hardware.configuration import load_layers
from mhseals_hardware.thruster_mixer import AXES, POSITIONS, mix_stages


def _row(values, width=7):
    return ' '.join(f'{value:+{width}.3f}' for value in values)


def _matrix(flat):
    return [tuple(flat[i:i + 3]) for i in range(0, len(flat), 3)]


def render(command, config):
    """The stages of ``command`` under ``config`` as plain text lines."""
    negative = config.get('decoupling_negative') or config['decoupling']
    stages = mix_stages(command, _matrix(config['thruster_matrix']), config['channel_map'],
                        config['reversed'], _matrix(config['decoupling']), _matrix(negative),
                        config['reverse_thrust_ratio'])
    ratio = config['reverse_thrust_ratio']
    lines = ['command        ' + '  '.join(f'{axis} {value:+.3f}'
                                          for axis, value in zip(AXES, stages['command'])),
             '',
             'L3 decoupling  part_k = C[:,k] * cmd_k  (C- column when cmd_k < 0)',
             '  axis    cmd     column  part_k (surge, sway, yaw)']
    for axis, value, column, part in zip(AXES, stages['command'], stages['columns'],
                                         stages['parts']):
        lines.append(f'  {axis:<6}{value:+7.3f}  {column:<6}{_row(part)}')
    lines += ['',
              'L2 geometry mix  u_ik = M_i . part_k;  T_i = sum_k u_ik|u_ik| (x r when < 0)',
              f'L1 thruster law  e_i = sgn(T_i) sqrt|T_i| (/ r when < 0), r = {ratio:g}',
              '  thr   M row (surge sway yaw)     u_surge  u_sway   u_yaw      T_i     e_i',
              ]
    mixer = _matrix(config['thruster_matrix'])
    for name, row, per_axis, thrust, raw in zip(POSITIONS, mixer, stages['per_axis'],
                                                stages['thrust'], stages['raw']):
        lines.append(f'  {name.upper():<4}{_row(row, 6)}    {_row(per_axis)} {thrust:+8.3f} {raw:+7.3f}')
    lines += [f'  desaturation: peak |e| {stages["peak"]:.3f} -> every e_i / {stages["peak"]:.3f}',
              '',
              'L0 PWM  reversed flips e; pulse = clip(1500 + 400 e, 1100, 1900); channel_map routes',
              '  thr   e_i     reversed  e_out   pulse  -> output']
    for name, effort, flip, out, pulse, channel in zip(
            POSITIONS, stages['effort'], config['reversed'], stages['flipped'], stages['pulses'],
            config['channel_map']):
        lines.append(f'  {name.upper():<4}{effort:+7.3f}   {str(flip):<8}{out:+7.3f}   {pulse:>5}  -> {channel}')
    lines.append('outputs 1-4 (us): ' + ' '.join(str(pulse) for pulse in stages['outputs']))
    return lines


def live(config):
    """Reprint on each change of topics.applied until Ctrl-C."""
    import rclpy
    from geometry_msgs.msg import Twist

    from mhseals_hardware.ros_util import twist_values

    rclpy.init()
    node = rclpy.create_node('mix_live')
    last = [None]

    def show(message):
        command = tuple(round(value, 3) for value in twist_values(message))
        if command != last[0]:
            last[0] = command
            print('\n'.join(['', f'--- {config["topics"]["applied"]}'] + render(command, config)),
                  flush=True)

    node.create_subscription(Twist, config['topics']['applied'], show, 10)
    print(f'following {config["topics"]["applied"]} (Ctrl-C ends)', flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('command', nargs='*', type=float, metavar='SURGE SWAY YAW',
                        help='normalized effort per axis, -1 to 1')
    parser.add_argument('--live', action='store_true', help='follow topics.applied')
    parser.add_argument('--config', help='hardware YAML (default: as the driver loads it)')
    parsed = parser.parse_args(args)
    if parsed.live == bool(parsed.command) or (parsed.command and len(parsed.command) != 3):
        parser.error('give SURGE SWAY YAW, or --live')
    try:
        config, source, _ = load_layers(parsed.config)
    except (OSError, ValueError) as error:
        print(f'mix: {error}', file=sys.stderr)
        return 1
    if parsed.live:
        live(config)
        return 0
    print('\n'.join([f'config: channel_map from {source}'] + render(tuple(parsed.command), config)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
