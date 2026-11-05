import contextlib
import io
from types import SimpleNamespace

import pytest
from rich.console import Console

from mhseals_hardware import thruster_test
from mhseals_hardware.thruster_mixer import MAX_PWM, MIN_PWM, NEUTRAL_PWM


class FakeOutputs:
    """Records every write; ``neutral()`` and ``close()`` both leave the outputs at neutral."""

    neutral_us = NEUTRAL_PWM

    def __init__(self):
        self.channels = [SimpleNamespace(path=f'/fake/pwm{i}') for i in range(4)]
        self.writes = []

    def open(self):
        self.writes.append([NEUTRAL_PWM] * 4)
        return self

    def set_pulse_widths(self, pulses):
        assert all(MIN_PWM <= p <= MAX_PWM for p in pulses), pulses
        self.writes.append(list(pulses))

    def neutral(self):
        self.writes.append([NEUTRAL_PWM] * 4)

    close = neutral


def keys(monkeypatch, sequence):
    class Reader:
        def __init__(self):
            self.keys = iter(sequence)

        def __enter__(self):
            return self

        def __exit__(self, *exc):
            return False

        def read(self, timeout):
            key = next(self.keys)
            if isinstance(key, BaseException):
                raise key
            return key

    monkeypatch.setattr(thruster_test, 'KeyReader', Reader)


def tui(outputs):
    return thruster_test.ThrusterTestTUI(outputs, Console(file=io.StringIO()))


@pytest.mark.parametrize('key', sorted(thruster_test.PRESETS))
def test_every_preset_stays_in_the_pwm_range(key):
    _, pulse = thruster_test.PRESETS[key]
    assert MIN_PWM <= pulse <= MAX_PWM
    outputs = FakeOutputs()
    test = tui(outputs)
    test.set_preset(key)
    test.adjust_pulse(1000)                 # nudging past the limit clamps
    test.adjust_pulse(-2000)
    assert outputs.writes[0] == [pulse] * 4


@pytest.mark.parametrize('end', ['q', 'escape', KeyboardInterrupt(), RuntimeError('boom')])
def test_any_exit_leaves_the_outputs_neutral(monkeypatch, end):
    outputs = FakeOutputs()
    keys(monkeypatch, ['down', 'x', 'right', None, 'f', end])
    raised = pytest.raises(type(end)) if isinstance(end, BaseException) else contextlib.nullcontext()
    with raised:
        tui(outputs).run()
    assert outputs.writes[-2] == [NEUTRAL_PWM + 100] + [NEUTRAL_PWM] * 3   # 'f' on physical 1 was live
    assert outputs.writes[-1] == [NEUTRAL_PWM] * 4                     # the exit wrote neutral


def test_arming_holds_neutral_and_locks_the_controls(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    monkeypatch.delenv('MHSEALS_HARDWARE_CONFIG', raising=False)
    outputs, events = FakeOutputs(), []
    monkeypatch.setattr(thruster_test.OdroidPWMOutputs, 'from_config',
                        classmethod(lambda cls, config: outputs))

    class Quiet(Console):
        def __init__(self):
            super().__init__(file=io.StringIO())

        def input(self, *args, **kwargs):
            return ''                        # Enter at the propeller warning

    monkeypatch.setattr(thruster_test, 'Console', Quiet)

    def sleep(seconds):
        events.append(('sleep', seconds, [list(w) for w in outputs.writes]))

    monkeypatch.setattr(thruster_test.time, 'sleep', sleep)
    keys(monkeypatch, ['x', 'q'])
    thruster_test.main([])
    (_, seconds, during), = events
    assert seconds == thruster_test.ARMING_SECONDS
    assert during == [[NEUTRAL_PWM] * 4]             # only the neutral open() before arming ends
    assert outputs.writes[1] == [MAX_PWM] * 4        # the controls work after arming
    assert outputs.writes[-1] == [NEUTRAL_PWM] * 4   # quit, then close, leave neutral

