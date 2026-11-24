"""Free the thruster pins: stop what holds them, neutral and disable each PWM output, release the MOSFET.

    thrusters free [--check]

Each output and the MOSFET line gets one row naming its holder. ``--check`` only reports. Exit 0 when
every row is free.
"""

import argparse
import os
from pathlib import Path
import signal
import sys
import time

from mhseals_hardware.configuration import load_config
from mhseals_hardware.odroid_pwm import (
    J2_PINS, MosfetEnable, pin_label, resolve_gpio_chip, resolve_pwm_chip,
)
from mhseals_hardware.thruster_mixer import NEUTRAL_PWM, POSITIONS

# The programs that open the pins: the driver, `thrusters SUB` (below) and the bench tools' old names.
NAMES = ('thruster_pwm_node', 'thruster_test', 'thruster_identify', 'mosfet_test')
GPIO_FDS = ('anon_inode:gpio-line', 'anon_inode:gpiohandle')
STOP_SECONDS = 3.0


def processes(proc='/proc', own=None):
    """``[(name, pid)]`` of the pin programs running here (``ros2 run`` scripts run as ``python3 SCRIPT``);
    a ``thrusters`` name carries its subcommand, and ``thrusters free`` is never one."""
    own = os.getpid() if own is None else own
    found = []
    for entry in Path(proc).glob('[0-9]*'):
        try:
            argv = (entry / 'cmdline').read_bytes().split(b'\0')
        except OSError:
            continue
        argv = [part.decode(errors='replace') for part in argv if part]
        if not argv or int(entry.name) == own:
            continue
        if os.path.basename(argv[0]).startswith('python') and len(argv) > 1:
            argv = argv[1:]
        name = os.path.basename(argv[0])
        if name == 'thrusters':
            name = f'thrusters {argv[1]}' if len(argv) > 1 and argv[1] != 'free' else None
        if name in NAMES or name and name.startswith('thrusters '):
            found.append((name, int(entry.name)))
    return sorted(found, key=lambda item: item[1])


def gpio_holders(procs, proc='/proc'):
    """The pin programs with a GPIO line open."""
    held = []
    for name, pid in procs:
        try:
            links = [os.readlink(fd) for fd in (Path(proc) / str(pid) / 'fd').iterdir()]
        except OSError:
            continue
        if any(link.startswith(GPIO_FDS) for link in links):
            held.append((name, pid))
    return held


def stop(procs, alive=lambda pid: Path(f'/proc/{pid}').exists(), kill=os.kill, sleep=time.sleep):
    """SIGINT each (they leave neutral, MOSFET off), SIGKILL what is left after STOP_SECONDS."""
    for name, pid in procs:
        print(f'stopping {name} {pid}')
        kill(pid, signal.SIGINT)
    deadline = time.monotonic() + STOP_SECONDS
    while any(alive(pid) for _, pid in procs) and time.monotonic() < deadline:
        sleep(0.1)
    for name, pid in procs:
        if alive(pid):
            print(f'killing {name} {pid} (no exit after {STOP_SECONDS:g} s)')
            kill(pid, signal.SIGKILL)


def channel_path(chip, channel):
    """The output's ``pwmN`` directory (never exported here), or None when the chip does not resolve."""
    try:
        return resolve_pwm_chip(chip) / f'pwm{channel}'
    except FileNotFoundError:
        return None


def read(path, name):
    try:
        return (path / name).read_text().strip()
    except OSError:
        return '?'


def release(config, mosfet=MosfetEnable):
    """Neutral, then disable, every configured output; request the MOSFET line inactive and release it."""
    for chip, channel in zip(config['pwm_chips'], config['pwm_channels']):
        path = channel_path(chip, channel)
        if path is None or not path.exists():
            continue
        try:
            (path / 'duty_cycle').write_text(str(NEUTRAL_PWM * 1000))
        except OSError:
            pass   # period 0: no duty fits, and a disabled output sends nothing anyway
        try:
            (path / 'enable').write_text('0')
        except OSError as error:
            print(f'{path}/enable: {error}')
    line = mosfet(config['mosfet_chip'], config['mosfet_line'], config['mosfet_active_high'])
    try:
        line.open()
    except Exception as error:   # held by a process outside this namespace, or no gpiod
        print(f'MOSFET: {error}')
    finally:
        line.close()


def line_info(chip, offset, gpiod=None):
    """``(used, consumer)`` of one GPIO line, libgpiod 2 or 1."""
    if gpiod is None:
        import gpiod
    handle = gpiod.Chip(resolve_gpio_chip(chip))
    try:
        if hasattr(handle, 'get_line_info'):
            info = handle.get_line_info(offset)
            return bool(info.used), info.consumer or ''
        line = handle.get_line(offset)
        return bool(line.is_used()), line.consumer() or ''
    finally:
        getattr(handle, 'close', lambda: None)()


def report(config, procs, gpiod=None, proc='/proc'):
    """``[(output, thruster, pin, device, enable, duty, holder)]``, outputs 1-4 then the MOSFET."""
    pwm_holders = [(n, p) for n, p in procs if n not in ('mosfet_test', 'thrusters mosfet')]
    rows = []
    for output, (chip, channel) in enumerate(zip(config['pwm_chips'], config['pwm_channels']), 1):
        thruster = POSITIONS[config['channel_map'].index(output)].upper()
        path = channel_path(chip, channel)
        if path is None:
            rows.append((str(output), thruster, pin_label(chip), str(chip), '-', '-', 'no chip'))
            continue
        enable, duty = (read(path, 'enable'), read(path, 'duty_cycle')) if path.exists() else ('-', '-')
        holder = ('free' if enable in ('0', '-') else
                  'held by ' + ', '.join(f'{n} {p}' for n, p in pwm_holders) if pwm_holders else
                  'held (enabled, no process)')
        rows.append((str(output), thruster, pin_label(chip), str(path), enable, duty, holder))
    pin = J2_PINS['mosfet']
    where = f'{config["mosfet_chip"]} line {config["mosfet_line"]}'
    try:
        used, consumer = line_info(config['mosfet_chip'], config['mosfet_line'], gpiod)
    except Exception as error:
        used, consumer = None, str(error)
    if used is None:
        holder = f'unknown ({consumer})'
    elif not used:
        holder = 'free'
    else:
        owners = gpio_holders(procs, proc)
        holder = 'held by ' + (', '.join(f'{n} {p}' for n, p in owners) if owners else consumer)
    rows.append(('MOSFET', '', f'J2 {pin[0]}', where, '', '', holder))
    return rows


def show(rows):
    header = ('output', 'thruster', 'pin', 'device', 'enable', 'duty', 'holder')
    widths = [max(len(row[i]) for row in (header, *rows)) for i in range(len(header))]
    for row in (header, *rows):
        print('  '.join(cell.ljust(width) for cell, width in zip(row, widths)).rstrip())


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--check', action='store_true', help='only report who holds what')
    parser.add_argument('--config', help='hardware YAML (or MHSEALS_HARDWARE_CONFIG)')
    parsed = parser.parse_args(args)
    config = load_config(parsed.config)
    if not parsed.check:
        stop(processes())
        release(config)
    rows = report(config, processes())
    show(rows)
    return 0 if all(row[-1] == 'free' for row in rows) else 1


if __name__ == '__main__':
    sys.exit(main())
