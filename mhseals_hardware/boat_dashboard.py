"""Pure Rich rendering for the commissioning dashboard (no ROS or GPIO)."""
from pathlib import PurePath
from rich.console import Group
from rich.layout import Layout
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

from mhseals_hardware.thruster_mixer import mix_thrusters

VIEWS = ('overview', 'sensors', 'tf', 'mapping', 'results', 'logs')
COLORS = {'LIVE': 'green', 'WARN': 'yellow', 'STALE': 'yellow', 'MISSING': 'red'}


def badge(status):
    return Text(status, style=COLORS.get(status, 'cyan'))


def sensor_table(snapshot, specs, compact=False):
    table = Table(expand=True, title='Received sensor data', padding=(0, 1))
    for column in (('Sensor', 'State', 'Hz') if compact else
                   ('Sensor / topic', 'State', 'Hz', 'Age', 'Details')):
        table.add_column(column, no_wrap=True, overflow='ellipsis')
    for name, topic, required in specs:
        value = snapshot['sensors'].get(name, {})
        status = value.get('status', 'MISSING')
        name_text = name + (' *' if required else '')
        row = [Text(name_text if compact else name_text + '\n' + topic),
               badge(status), f"{value.get('hz', 0):.1f}"]
        if not compact:
            values = value.get('values', {})
            detail = value.get('issue') or values.get('summary', '')
            row += [f"{value['age']:.2f}s" if 'age' in value else '--', Text(detail)]
        table.add_row(*row)
    return table


def motion_panel(snapshot, context):
    command = context.get('command', (0, 0, 0))
    odom = snapshot['sensors'].get('odometry', {})
    measured = odom.get('values', {})
    table = Table.grid(expand=True, padding=(0, 1))
    table.add_row('', 'SURGE +forward', 'SWAY +port', 'YAW +CCW')
    table.add_row('Effort command', *(f'{v:+.0%}' for v in command))
    table.add_row('Measured', *(f'{measured[k]:+.3f}' if measured.get(k) is not None else '--'
                               for k in ('vx', 'vy', 'wz')))
    table.add_row('Units', 'm/s', 'm/s', 'rad/s')
    def trend(key):
        values = snapshot.get('trends', {}).get(key, [])
        if not values:
            return '--'
        values = values[::max(1, len(values) // 12)][-12:]
        low, high = min(values), max(values)
        return ('─' * len(values) if high - low < .001 else
                ''.join('▁▂▃▄▅▆▇█'[min(7, int(7 * (v-low)/(high-low)))] for v in values))
    table.add_row('Recent trend*', *(trend(key) for key in ('vx', 'vy', 'wz')))
    frame = measured.get('child', 'unknown')
    return Panel(Group(table, Text(f"Odometry: {odom.get('status', 'MISSING')} | frame: {frame} | *auto-scaled")),
                 title='Motion — effort is NOT velocity', border_style='cyan')


def mapping_panel(context, compact=False):
    mapping = context.get('channel_map', (1, 2, 3, 4))
    command = context.get('command', (0, 0, 0))
    outputs = mix_thrusters(*command, mixer=context['matrix'])
    labels = {position: f'{position.upper()} → #{channel}  {output if output else 0:+.0%}'
              for position, channel, output in zip(('fl', 'fr', 'rr', 'rl'), mapping, outputs)}
    hull = Text(f"           BOW / +surge\n{labels['fl']}     {labels['fr']}\n"
                f"     │                   │\n{labels['rl']}     {labels['rr']}\n"
                'Predicted mixer effort; not motor feedback', style='cyan')
    if compact:
        return Panel(hull, title='Thruster map', border_style='blue')
    table = Table(expand=True)
    for name in ('Position', 'Output', 'PWM chip', 'Channel', 'Readback'):
        table.add_column(name, no_wrap=True, overflow='ellipsis')
    for position, channel in zip(('FL', 'FR', 'RR', 'RL'), mapping):
        readback = context.get('pwm_readback', ['not sampled'] * 4)[channel - 1]
        if readback == 'unavailable / unexported':
            readback = 'unavailable'
        path = PurePath(context['chips'][channel - 1])
        chip = next((part for part in path.parts if part.endswith('.pwm')), path.name)
        table.add_row(position, str(channel), Text(chip),
                      str(context['channels'][channel - 1]), Text(readback))
    return Group(Panel(hull, title='Thruster mapping'), table,
                 Text('R edit & persist mapping (no pulses)   I guided pulse identification\n'
                      'Both actions disarm first. Pin overrides: --config / --pwm-chips / --pwm-channels.'))


def tf_table(snapshot):
    table = Table(expand=True, title='TF chain integrity')
    for column in ('Required chain', 'State', 'Details'):
        table.add_column(column)
    for edge in snapshot['tf']:
        table.add_row(Text(edge['source'] + ' → ' + edge['target']),
                      badge(edge['status']), Text(edge['detail']))
    return Group(table, Text('Checks freshness, reachability, cycles and competing publishers.\n'
                             'A green chain does not verify physical mounting, axes or calibration.',
                             style='dim'))


def result_table(snapshot):
    table = Table(expand=True, title='Measured trial results (newest first)')
    for column in ('Axis', 'Verdict', 'Samples', 'Baseline', 'Mean', 'Response'):
        table.add_column(column)
    for result in reversed(snapshot['results'][-10:]):
        table.add_row(result['axis'] + (' +' if result['direction'] > 0 else ' -'),
                      Text(result['verdict']), str(result['samples']),
                      *(f'{result[k]:+.3f}' if k in result else '--'
                        for k in ('baseline', 'mean', 'response')))
    return Group(table, Text('Response = mean velocity minus pre-test drift. SIGN MATCH is not calibration.\n'
                             'Surge/sway: m/s. Yaw: rad/s. JSON report includes trial details.'))


def render_dashboard(snapshot, context, width=100, height=32):
    view = context.get('view', 'overview')
    armed = context.get('armed', False)
    header = Text(f" BOAT TEST  {context.get('host', '')}  ROS {context.get('domain', '?')}   ")
    header.append('ARMED' if armed else 'DISARMED / MONITOR', style='bold red' if armed else 'bold green')
    header.append(f"   BAG: {context.get('bag', 'OFF')}\n ")
    for name in VIEWS:
        header.append(f' {name.upper()} ', style='bold black on cyan' if name == view else 'dim')
    specs = context['specs']
    if view == 'sensors':
        body = sensor_table(snapshot, specs)
    elif view == 'tf':
        body = tf_table(snapshot)
    elif view == 'mapping':
        body = mapping_panel(context)
    elif view == 'results':
        body = result_table(snapshot)
    elif view == 'logs':
        body = Group(Text('Recent events', style='bold'),
                     *(Text(message) for _, message in snapshot['events']),
                     Text('\nProcesses / logs\n' + context.get('process_summary', 'None')))
    else:
        right = Group(motion_panel(snapshot, context), mapping_panel(context, compact=True))
        if width >= 100:
            body = Layout()
            body.split_row(Layout(sensor_table(snapshot, specs, compact=True), ratio=1),
                           Layout(Text(''), size=1),
                           Layout(right, ratio=2))
        else:
            states = '  '.join(f"{name}: {snapshot['sensors'].get(name, {}).get('status', 'MISSING')}"
                               for name, _, required in specs if required)
            body = Group(Text(states), right)
    tf_ok = all(row['status'] == 'LIVE' for row in snapshot['tf'])
    override = 'SENSOR OVERRIDE | ' if context.get('sensor_override') else ''
    footer = Text(f"TF: {'LIVE' if tf_ok else 'CHECK'} | {override}{context.get('notice', 'Ready')}\n",
                  no_wrap=True, overflow='ellipsis')
    footer.append(('WASD move  Arrows surge/yaw  +/- effort\nSpace STOP  M/X/Esc return to dashboard'
                   if context.get('manual') else
                   'Tab views  H arm/disarm  M manual  1/2/3 tests  +/- effort\n'
                   'R remap  I identify  B record  E export  Space STOP  Q quit'), style='bold')
    layout = Layout()
    layout.split_column(Layout(header, size=3), Layout(body), Layout(footer, size=4))
    return layout
