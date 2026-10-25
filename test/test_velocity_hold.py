"""A command_mux hold means neutral thrusters, not station-keeping on the mux's zeros."""
import json
from types import SimpleNamespace as NS

import numpy
import pytest

from mhseals_hardware.velocity_law import Plant, VelocityLaw
from test_command_arbitration import LOW, PUSH, boat, frame
from test_controller_parameters import node
from test_velocity_law import config


def mux(n, source):
    n._mux_state(NS(data=json.dumps({'input': source, 'cause': 'test'})))


def pushing():
    """A controller holding still against a current, with the integral in use."""
    n = node()
    n._law.drift[:] = (0.0, 0.1)
    n._law.integral[:] = (0.1, -0.2, 0.05)
    n._law.observe((0., 0., 0.), 0., 1.)
    n._command((0., 0., 0.))
    assert any(n.sent[-1])  # station-keeping pushes
    return n


@pytest.mark.parametrize('source', ['hold', 'idle', None])
def test_a_hold_gives_zero_effort_at_once_and_while_it_lasts(source):
    n = pushing()
    if source is None:
        n._mux_state(NS(data='not json'))  # unreadable: neutral
    else:
        mux(n, source)
    assert n.sent[-1] == (0., 0., 0.)  # published by the state itself, not the next frame
    assert not n._law.integral.any() and not n._law.effort.any()
    n._command((0., 0., 0.))  # the mux keepalive while it holds
    assert n.sent[-1] == (0., 0., 0.) and not n._law.integral.any()
    assert n._law.drift == pytest.approx((0.0, 0.1))  # the current estimate is kept


def test_a_hold_is_neutral_at_once_while_the_input_still_reads_manual():
    n = pushing()
    mux(n, 'manual')
    n._command((0., 0., 0.))
    assert any(n.sent[-1])                      # the keyboard drives (station-keeps here)
    # Space: the hold is set, but manual stays the input until its frames time out.
    n._mux_state(NS(data=json.dumps({'input': 'manual', 'since_ns': 100,
                                     'holds': [{'cause': 'keys', 'since_ns': 200}]})))
    assert n.sent[-1] == (0., 0., 0.) and not n._law.integral.any()
    n._command((0., 0., 0.))                    # the first frame of the hold
    assert n.sent[-1] == (0., 0., 0.)


@pytest.mark.parametrize('source', ['manual', 'rc'])
def test_a_takeover_after_a_hold_still_drives(source):
    n, plain = pushing(), pushing()
    n._mux_state(NS(data=json.dumps({'input': source, 'since_ns': 300,
                                     'holds': [{'cause': 'rc_stick', 'since_ns': 200}]})))
    n._command((.12, 0., 0.))
    plain._command((.12, 0., 0.))
    assert n.sent[-1] == pytest.approx(plain.sent[-1]) and any(n.sent[-1])


def test_a_hold_without_a_start_time_is_neutral():
    n = pushing()
    n._mux_state(NS(data=json.dumps({'input': 'manual', 'since_ns': 300,
                                     'holds': [{'cause': 'a', 'since_ns': 100},
                                               {'cause': 'b'}]})))
    assert n.sent[-1] == (0., 0., 0.)


# Through the real mux state machine, published as the mux node does.

def publish(n, a, now):
    """Feed ``a``'s ``~/state`` at ``now`` to the controller; return the input."""
    state = a.state(now, wall_ns=10**18 + int(now * 1e9))
    n._mux_state(NS(data=json.dumps(state)))
    return state['input']


def drives(n, plain):
    n._command((.12, 0., 0.))
    plain._command((.12, 0., 0.))
    return any(n.sent[-1]) and n.sent[-1] == pytest.approx(plain.sent[-1])


def test_a_new_hold_behind_an_older_one_is_neutral_on_the_first_cycle():
    n, plain, a = pushing(), pushing(), boat()
    a.request('hold', 'op', 'tui', 0.0)                 # hold A
    a.update_manual((.12, 0., 0.), 0.1)                 # the keyboard takes over
    assert publish(n, a, 0.1) == 'manual' and drives(n, plain)
    a.request('hold', 'space', 'tui', 0.2)              # Space: hold B
    assert publish(n, a, 0.2) == 'manual'               # still reads manual
    assert a.state(0.2)['hold']['cause'] == 'op'        # the oldest hold is A
    n._command((.12, 0., 0.))
    assert n.sent[-1] == (0., 0., 0.)


def test_placing_the_same_cause_again_is_neutral():
    n, plain, a = pushing(), pushing(), boat()
    a.request('hold', 'space', 'tui', 0.0)
    a.update_manual((.12, 0., 0.), 0.1)
    assert publish(n, a, 0.1) == 'manual' and drives(n, plain)
    a.request('hold', 'space', 'tui', 0.2)              # Space again
    assert publish(n, a, 0.2) == 'manual'
    n._command((.12, 0., 0.))
    assert n.sent[-1] == (0., 0., 0.)


def test_rc_driving_keeps_driving_through_a_later_hold():
    n, plain, a = pushing(), pushing(), boat()
    a.update_rc(frame(mode=LOW, sticks=PUSH), 0.0)      # switch at MANUAL: rc drives
    assert publish(n, a, 0.0) == 'rc' and drives(n, plain)
    a.request('hold', 'space', 'tui', 0.1)
    a.update_rc(frame(mode=LOW, sticks=PUSH), 0.1)
    assert publish(n, a, 0.1) == 'rc' and a.holds
    assert drives(n, plain)                             # rc outranks holds


def test_rc_takeover_after_a_hold_drives():
    n, a = pushing(), boat()
    a.request('hold', 'space', 'tui', 0.0)
    assert publish(n, a, 0.0) == 'hold'
    a.update_rc(frame(mode=LOW, sticks=PUSH), 0.1)
    assert publish(n, a, 0.1) == 'rc'
    n._command((.12, 0., 0.))                           # the law was reset by the hold
    assert any(n.sent[-1])


def test_release_to_auto_resumes_with_no_stale_integral():
    n = pushing()
    mux(n, 'hold')
    n._command((0., 0., 0.))
    mux(n, 'auto')
    n._command((.12, 0., 0.))
    fresh = VelocityLaw(Plant(config()))
    fresh.drift[:] = (0.0, 0.1)
    fresh.observe((0., 0., 0.), 0., 1.)
    assert n.sent[-1] == pytest.approx(fresh.step((.12, 0., 0.), 1.))


@pytest.mark.parametrize('source', ['auto', 'manual', 'rc'])
def test_driving_sources_are_unchanged(source):
    n, plain = pushing(), pushing()
    mux(n, source)
    for command in ((.12, 0., 0.), (0., .05, .1)):
        n._command(command)
        plain._command(command)
        assert n.sent[-1] == pytest.approx(plain.sent[-1])
    assert numpy.any(n.sent[-1])
