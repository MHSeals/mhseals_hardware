"""Flight controller parameters from named lists: show, apply (with a backup and a read-back) and undo.

Lists live in config/fcu_params.yaml (and ~/.config/mhseals/fcu_params.yaml, which adds or replaces lists by
name). Values go through MAVROS's param plugin, which mirrors the FCU's parameters on the node /mavros/param.
Each apply appends {time, list, values: {NAME: [old, new]}} to ~/.config/mhseals/fcu_params.history.jsonl;
undo restores one entry's old values (picked newest first, or --last) and reads them back.
"""
import argparse
import datetime
import json
import os
import sys
from pathlib import Path

import yaml

NODE = '/mavros/param'
HOME = Path(os.environ.get('MHSEALS_CONFIG', Path.home() / '.config' / 'mhseals'))
HISTORY = HOME / 'fcu_params.history.jsonl'


def package_file():
    here = Path(__file__).resolve().parent.parent / 'config' / 'fcu_params.yaml'
    if here.exists():
        return here
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('mhseals_hardware')) / 'config' / 'fcu_params.yaml'


def load_lists(paths=None):
    """{name: {description, reboot, params}}: the package's lists, then the user's (same name replaces)."""
    out = {}
    for path in paths or (package_file(), HOME / 'fcu_params.yaml'):
        path = Path(path)
        if path.exists():
            data = yaml.safe_load(path.read_text()) or {}
            for name, entry in (data.get('lists') or {}).items():
                if not isinstance(entry, dict) or not isinstance(entry.get('params'), dict):
                    raise ValueError(f'{path}: lists.{name} needs params: {{NAME: value}}')
                out[name] = entry
    return out


def history(path=None):
    path = path or HISTORY
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def record(entry, path=None):
    path = path or HISTORY
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'a') as out:
        out.write(json.dumps(entry) + '\n')


def same(a, b):
    return a is not None and b is not None and abs(float(a) - float(b)) < 1e-6


class Fcu:
    """Get and set FCU parameters through /mavros/param's parameter services."""

    def __init__(self, timeout=5.0):
        import rclpy
        self.rclpy, self.timeout = rclpy, timeout
        rclpy.init()
        self.node = rclpy.create_node('astro_fcu_params')

    def _call(self, kind, request):
        from rcl_interfaces.srv import GetParameters, SetParameters
        service = {'get': GetParameters, 'set': SetParameters}[kind]
        client = self.node.create_client(service, f'{NODE}/{kind}_parameters')
        if not client.wait_for_service(timeout_sec=self.timeout):
            raise RuntimeError(f'{NODE} does not answer (is MAVROS up? astro up localization)')
        future = client.call_async(request)
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.timeout)
        if not future.done():
            raise RuntimeError(f'{NODE} did not answer {kind}_parameters')
        return future.result()

    def get(self, names):
        """{NAME: number or None (the FCU has no such parameter)}."""
        from rcl_interfaces.msg import ParameterType
        from rcl_interfaces.srv import GetParameters
        result = self._call('get', GetParameters.Request(names=list(names)))
        out = {}
        for name, value in zip(names, result.values):
            out[name] = {ParameterType.PARAMETER_INTEGER: value.integer_value,
                         ParameterType.PARAMETER_DOUBLE: value.double_value}.get(value.type)
        return out

    def set(self, values, current):
        """Set {NAME: number}, each with the type the FCU reports; returns {NAME: reason} for refusals."""
        from rclpy.parameter import Parameter
        from rcl_interfaces.srv import SetParameters
        params = []
        for name, value in values.items():
            if isinstance(current.get(name), int) and float(value).is_integer():
                params.append(Parameter(name, Parameter.Type.INTEGER, int(value)).to_parameter_msg())
            else:
                params.append(Parameter(name, Parameter.Type.DOUBLE, float(value)).to_parameter_msg())
        result = self._call('set', SetParameters.Request(parameters=params))
        return {name: r.reason for name, r in zip(values, result.results) if not r.successful}

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


def write(fcu, values, label, note):
    """Back up, set, read back; prints NAME old -> new and records the change. Returns an exit code."""
    current = fcu.get(list(values))
    missing = [n for n, v in current.items() if v is None]
    if missing:
        print(f'the flight controller has no {", ".join(missing)} (wrong firmware or name?): nothing written')
        return 1
    changes = {n: v for n, v in values.items() if not same(current[n], v)}
    if not changes:
        print('every value is already set: nothing written')
        return 0
    refused = fcu.set(changes, current)
    after = fcu.get(list(changes))
    done = {n: [current[n], after[n]] for n in changes if same(after[n], changes[n])}
    for name in changes:
        mark = 'ok' if name in done else f'NOT SET ({refused.get(name) or "read back " + str(after[name])})'
        print(f'  {name}  {current[name]} -> {changes[name]}   {mark}')
    if done:
        record(dict(time=datetime.datetime.now().astimezone().isoformat(timespec='seconds'), list=label,
                    values=done))
    if note:
        print(note)
    return 0 if len(done) == len(changes) else 1


def show(fcu, lists, name=None):
    chosen = {name: lists[name]} if name else lists
    names = sorted({p for entry in chosen.values() for p in entry['params']})
    current = fcu.get(names) if names else {}
    for list_name, entry in chosen.items():
        state = all(same(current.get(p), v) for p, v in entry['params'].items())
        print(f'{list_name}{"  (in force)" if state else ""}: {entry.get("description", "")}')
        for p, v in entry['params'].items():
            print(f'  {p}  now {current.get(p)}  list {v}{"" if same(current.get(p), v) else "   differs"}')
    return 0


def pick(entries, last):
    """The history entry to undo: the newest with --last, else asked on a terminal; None to cancel."""
    if last:
        return entries[-1]
    for i, entry in enumerate(reversed(entries), 1):
        values = ', '.join(f'{n} {old} -> {new}' for n, (old, new) in entry['values'].items())
        print(f'  {i}  {entry["time"]}  {entry["list"]}: {values}')
    if not sys.stdin.isatty():
        print('no terminal: pass --last to undo the newest')
        sys.exit(2)
    answer = input('undo which (number, Enter for 1, q to cancel)? ').strip() or '1'
    if answer.lower().startswith('q'):
        return None
    return list(reversed(entries))[int(answer) - 1]


def main(argv=None):
    parser = argparse.ArgumentParser(prog='fcu_params', description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest='action', required=True)
    sub.add_parser('lists')
    s = sub.add_parser('show')
    s.add_argument('list', nargs='?')
    a = sub.add_parser('apply')
    a.add_argument('list')
    u = sub.add_parser('undo')
    u.add_argument('--last', action='store_true')
    u.add_argument('--list', dest='listing', action='store_true')
    args = parser.parse_args(argv)
    lists = load_lists()
    if args.action == 'lists':
        for name, entry in lists.items():
            print(f'{name}\t{entry.get("description", "")}')
        return 0
    if args.action == 'undo' and args.listing:
        for entry in reversed(history()):
            print(f'{entry["time"]}  {entry["list"]}: ' +
                  ', '.join(f'{n} {old} -> {new}' for n, (old, new) in entry['values'].items()))
        return 0
    if getattr(args, 'list', None) and args.list not in lists:
        print(f'no list {args.list!r} (lists: {", ".join(lists)})')
        return 2
    fcu = Fcu()
    try:
        if args.action == 'show':
            return show(fcu, lists, args.list)
        if args.action == 'apply':
            entry = lists[args.list]
            note = 'power cycle the flight controller for this to take effect' if entry.get('reboot') else None
            return write(fcu, entry['params'], args.list, note)
        entries = history()
        if not entries:
            print('nothing to undo')
            return 0
        chosen = pick(entries, args.last)
        if chosen is None:
            print('cancelled, nothing written')
            return 1
        print(f'undo {chosen["time"]} {chosen["list"]}:')
        return write(fcu, {n: old for n, (old, new) in chosen['values'].items()}, f'undo {chosen["list"]}',
                     'power cycle the flight controller if the list needed it')
    except RuntimeError as error:
        print(f'fcu params: {error}')
        return 1
    finally:
        fcu.close()


if __name__ == '__main__':
    sys.exit(main())
