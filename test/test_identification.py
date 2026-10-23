"""Thruster identification and the option picker, with fake outputs and keys."""
from types import SimpleNamespace

import pytest

from mhseals_hardware import identification
from mhseals_hardware.identification import Cancelled, identify_thrusters
from mhseals_hardware.menu import select_option
from mhseals_hardware.thruster_mixer import NEUTRAL_PWM, effort_to_pulse


class FakeOutputs:
    def __init__(self):
        self.writes = []

    def set_pulse_widths(self, pulses):
        self.writes.append(list(pulses))

    def neutral(self):
        self.writes.append([NEUTRAL_PWM] * 4)


def harness(answers, keys=()):
    """Fake clock, key reader and operator; physical channel N maps to answers[N-1]."""
    clock = SimpleNamespace(now=0.0)
    pending = list(keys)

    class Keys:
        def __enter__(self):
            return self

        def __exit__(self, *_):
            pass

        def read(self, timeout):
            clock.now += timeout
            return pending.pop(0) if pending else None

    replies = iter(answers)
    events = []
    run = lambda outputs: identify_thrusters(  # noqa: E731
        outputs, prompt=lambda message: None,
        choose=lambda title, options, shortcuts: next(replies),
        emit=lambda event, **details: events.append((event, details)),
        open_keys=Keys, clock=lambda: clock.now)
    return run, events


def test_identifies_a_swapped_wiring_and_pulses_one_output_at_a_time():
    # Physical 1..4 are wired to FR, FL, RL, RR; every answer is CCW and polarity "yes".
    answers = []
    for position in ('fr', 'fl', 'rl', 'rr'):
        answers += [position, 'CCW', 'yes']
    run, events = harness(answers)
    outputs = FakeOutputs()
    channel_map, rotations, reversed = run(outputs)
    assert channel_map == (2, 1, 4, 3) and reversed == (False,) * 4  # canonical FL, FR, RR, RL -> physical outputs
    assert rotations == {position: 'CCW' for position in ('fl', 'fr', 'rr', 'rl')}
    pulse = effort_to_pulse(identification.IDENTIFY_EFFORT)
    for channel in range(4):
        expected = [NEUTRAL_PWM] * 4
        expected[channel] = pulse
        assert expected in outputs.writes  # each output pulsed alone
    assert not any(sum(value != NEUTRAL_PWM for value in write) > 1 for write in outputs.writes)
    assert outputs.writes[-1] == [NEUTRAL_PWM] * 4
    assert [event for event, _ in events].count('identification_result') == 4


def test_a_backwards_thruster_is_recorded_as_reversed():
    answers = []
    for position, polarity in (('fl', 'no'), ('fr', 'yes'), ('rr', 'yes'), ('rl', 'no')):
        answers += [position, 'CW', polarity]
    run, events = harness(answers)
    outputs = FakeOutputs()
    channel_map, _, reversed = run(outputs)
    assert channel_map == (1, 2, 3, 4) and reversed == (True, False, False, True)
    assert outputs.writes[-1] == [NEUTRAL_PWM] * 4
    assert [d['reversed'] for e, d in events if e == 'identification_result'] == [
        True, False, False, True]


@pytest.mark.parametrize('key', ['Q', 'SPACE'])
def test_caps_lock_still_aborts(key):
    run, _ = harness([], keys=[None, None, key])
    outputs = FakeOutputs()
    with pytest.raises(Cancelled):
        run(outputs)
    assert outputs.writes[-1] == [NEUTRAL_PWM] * 4


def test_space_during_a_pulse_cancels_at_neutral():
    # The first hold is the arming period; Space arrives during it.
    run, _ = harness([], keys=[None, None, 'space'])
    outputs = FakeOutputs()
    with pytest.raises(Cancelled):
        run(outputs)
    assert outputs.writes[-1] == [NEUTRAL_PWM] * 4


def test_select_option_arrows_shortcuts_and_cancel():
    def picker(keys):
        pending = list(keys)

        class Keys:
            def __enter__(self):
                return self

            def __exit__(self, *_):
                pass

            def read(self, timeout):
                return pending.pop(0) if pending else 'q'

        class View(Keys):
            def __init__(self, *args, **kwargs):
                pass

            def update(self, *_):
                pass

        return select_option(None, 'title', ('a', 'b', 'c'), {'x': 2}, open_keys=Keys, live=View)

    assert picker(['down', 'down', 'up', 'enter']) == 'b'
    assert picker(['x']) == 'c'
    assert picker(['up', 'enter']) == 'c'  # wraps around
    with pytest.raises(Cancelled):
        picker(['escape'])


def test_choose_mapping_assigns_each_output_once():
    from mhseals_hardware.identification import choose_mapping
    picks = iter(['2', '1', '4', '3'])
    seen = []

    def choose(title, options, shortcuts):
        seen.append(options)
        return next(picks)

    assert choose_mapping(choose) == (2, 1, 4, 3)
    assert seen[-1] == ('3',)  # each output can only be chosen once


def test_save_mapping_persists_only_the_channel_map(tmp_path):
    import yaml
    from mhseals_hardware.identification import save_mapping
    path = tmp_path / 'hardware.yaml'
    path.write_text('frequency: 50.0\n')
    console = type('Console', (), {'print': lambda self, *a: None,
                                    'input': lambda self, prompt: 'y'})()
    save_mapping(console, (2, 1, 4, 3), path, (True, False, False, False))
    saved = yaml.safe_load(path.read_text())
    assert saved['channel_map'] == [2, 1, 4, 3] and saved['output_backend'] == 'odroid'
    assert saved['reversed'] == [True, False, False, False]
