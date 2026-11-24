"""thrusters free: who holds the pins (fake /proc, sysfs and libgpiod), and releasing them."""

from types import SimpleNamespace
import os

from mhseals_hardware import free
from mhseals_hardware.configuration import load_defaults


def fake_proc(root, pid, argv, gpio=False):
    entry = root / str(pid)
    (entry / 'fd').mkdir(parents=True)
    (entry / 'cmdline').write_bytes(b'\0'.join(part.encode() for part in argv) + b'\0')
    if gpio:
        os.symlink('anon_inode:gpio-line', entry / 'fd' / '7')


def fake_pins(tmp_path):
    chips = []
    for n in range(4):
        pwm = tmp_path / 'sys' / f'chip{n}.pwm' / 'pwm' / f'pwmchip{n}' / 'pwm0'
        pwm.mkdir(parents=True)
        (pwm / 'enable').write_text('1')
        (pwm / 'duty_cycle').write_text('1700000')
        chips.append(str(pwm.parent.parent / 'pwmchip*'))
    return dict(load_defaults(), pwm_chips=chips, mosfet_chip='/dev/gpiochip5')


def gpiod(used):
    info = SimpleNamespace(used=used[0], consumer='mhseals-thruster-mosfet')
    return SimpleNamespace(Chip=lambda path: SimpleNamespace(get_line_info=lambda offset: info))


def test_report_names_each_holder_and_release_frees_every_row(tmp_path):
    proc = tmp_path / 'proc'
    fake_proc(proc, 1234, ['/usr/bin/python3', '/ws/lib/mhseals_hardware/thruster_pwm_node', '--ros-args'], gpio=True)
    fake_proc(proc, 1300, ['/usr/bin/python3', '/ws/lib/mhseals_hardware/thrusters', 'free'])
    fake_proc(proc, 1400, ['/usr/bin/python3', '/ws/lib/mhseals_hardware/thrusters', 'mosfet'])
    fake_proc(proc, 1500, ['bash'])
    procs = free.processes(proc, own=1)
    assert procs == [('thruster_pwm_node', 1234), ('thrusters mosfet', 1400)]
    config, used = fake_pins(tmp_path), [True]
    rows = free.report(config, procs, gpiod(used), proc)
    assert rows[0][-1] == 'held by thruster_pwm_node 1234' and rows[0][1] == 'FL'
    assert rows[-1][-1] == 'held by thruster_pwm_node 1234'

    lines = []

    class Line:
        def __init__(self, *args):
            pass

        def open(self):
            lines.append('request inactive')

        def close(self):
            lines.append('release')
            used[0] = False
    free.release(config, mosfet=Line)
    rows = free.report(config, [], gpiod(used), proc)
    assert lines == ['request inactive', 'release']
    assert all(row[-1] == 'free' for row in rows)
    assert rows[0][4:6] == ('0', '1500000')
