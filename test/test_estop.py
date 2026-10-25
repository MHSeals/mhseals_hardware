"""Remote e-stop: the RC stop edge cuts the MOSFET and latches until a new driver arms."""

from types import SimpleNamespace
import sys

import pytest

from mhseals_hardware.configuration import load_defaults
from mhseals_hardware.estop import RemoteEstop
from mhseals_hardware.odroid_pwm import OdroidPWMOutputs

ON, OFF = 2000, 1000      # stop channel: run, stop


def frame(stop):
    """A valid frame: sticks centered, mode switch at MANUAL, channel 6 the stop switch."""
    channels = [1500] * 8
    channels[5], channels[7] = stop, 1000
    return channels


@pytest.fixture
def line(monkeypatch):
    """A fake libgpiod 2 bank recording every level driven on the enable line."""
    levels = []

    class Request:
        def set_value(self, offset, value):
            levels.append(value)

        def release(self):
            levels.append('release')

    class Chip:
        def __init__(self, path):
            pass

        def request_lines(self, **kwargs):
            levels.append(kwargs['config'][29].output_value)
            return Request()

        def close(self):
            pass

    monkeypatch.setitem(sys.modules, 'gpiod', SimpleNamespace(
        Chip=Chip, LineSettings=SimpleNamespace,
        line=SimpleNamespace(Value=SimpleNamespace(ACTIVE=1, INACTIVE=0),
                             Direction=SimpleNamespace(OUTPUT=3))))
    return levels


def fake_chip(root, number):
    pwm = root / f'pwmchip{number}' / 'pwm0'
    pwm.mkdir(parents=True)
    for name in ('enable', 'duty_cycle', 'period', 'polarity'):
        (pwm / name).write_text('0', encoding='ascii')
    return pwm.parent


def arm(tmp_path, enabled=True):
    """What the arm path does: open fresh outputs (MOSFET on at neutral), a new e-stop."""
    chips = [fake_chip(tmp_path / str(len(list(tmp_path.iterdir()))), n) for n in range(4)]
    outputs = OdroidPWMOutputs(chips, mosfet_chip='/dev/gpiochip5', mosfet_line=29).open()
    return outputs, RemoteEstop(outputs, load_defaults()['command_mux'], enabled)


def duties(outputs):
    return [(c.path / 'duty_cycle').read_text() for c in outputs.channels]


def test_stop_edge_cuts_power_then_neutral_and_latches(tmp_path, line):
    outputs, estop = arm(tmp_path)
    assert line == [0, 1] and outputs.mosfet.enabled
    outputs.set_pulse_widths([1700] * 4)
    assert not estop.rc(frame(ON), 1.0, 10)
    assert estop.rc(frame(OFF), 1.02, 20)
    assert line[-1] == 0 and not outputs.mosfet.enabled
    assert duties(outputs) == ['1500000'] * 4
    assert estop.state() == {'enabled': True, 'latched': True, 'cause': 'rc-stop',
                             'since_ns': 20}
    assert outputs.mosfet.request is not None  # still held low, not released to float


def test_rc_returning_never_powers_on(tmp_path, line):
    outputs, estop = arm(tmp_path)
    estop.rc(frame(ON), 1.0, 0)
    estop.rc(frame(OFF), 1.02, 0)
    for step in range(10):     # switch back to run, toggle again, keep streaming
        estop.rc(frame(ON if step % 2 == 0 else OFF), 1.04 + step * 0.02, 0)
    assert line.count(1) == 1 and line[-1] == 0
    assert estop.latched and estop.state()['since_ns'] == 0


def test_only_a_new_arm_restores_power(tmp_path, line):
    outputs, estop = arm(tmp_path)
    estop.rc(frame(ON), 1.0, 0)
    estop.rc(frame(OFF), 1.02, 0)
    outputs.close()            # disarm: the driver exits
    line.clear()
    outputs, estop = arm(tmp_path)
    assert line == [0, 1] and not estop.latched
    assert not estop.rc(frame(OFF), 2.0, 0)   # a switch already at stop is no edge
    assert outputs.mosfet.enabled


def test_invalid_frames_do_not_cut(tmp_path, line):
    """The mux's own rule: only a valid frame after a run frame is an edge."""
    outputs, estop = arm(tmp_path)
    estop.rc(frame(ON), 1.0, 0)
    assert not estop.rc(frame(OFF)[:3], 1.02, 0)          # too few channels
    garbled = frame(OFF)
    garbled[0] = 0
    estop.rc(frame(ON), 1.04, 0)
    assert not estop.rc(garbled, 1.06, 0)                 # out-of-range stick
    assert outputs.mosfet.enabled and not estop.latched


def test_disabled_keeps_power(tmp_path, line):
    outputs, estop = arm(tmp_path, enabled=False)
    estop.rc(frame(ON), 1.0, 0)
    assert not estop.rc(frame(OFF), 1.02, 0)
    assert outputs.mosfet.enabled and estop.state()['enabled'] is False


def test_failed_cut_still_latches_and_tries_neutral(tmp_path, line):
    outputs, estop = arm(tmp_path)
    outputs.set_pulse_widths([1700] * 4)

    def broken(enabled):
        raise OSError('line busy')
    outputs.mosfet.set_enabled = broken
    estop.rc(frame(ON), 1.0, 0)
    with pytest.raises(OSError):
        estop.rc(frame(OFF), 1.02, 0)
    assert estop.latched and duties(outputs) == ['1500000'] * 4
