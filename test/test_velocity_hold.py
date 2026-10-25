"""A command_mux hold means neutral thrusters, not station-keeping on the mux's zeros."""
import json
from types import SimpleNamespace as NS

import numpy
import pytest

from mhseals_hardware.velocity_law import Plant, VelocityLaw
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
                                     'hold': {'cause': 'keys', 'since_ns': 200}})))
    assert n.sent[-1] == (0., 0., 0.) and not n._law.integral.any()
    n._command((0., 0., 0.))                    # the first frame of the hold
    assert n.sent[-1] == (0., 0., 0.)


@pytest.mark.parametrize('source', ['manual', 'rc'])
def test_a_takeover_after_a_hold_still_drives(source):
    n, plain = pushing(), pushing()
    n._mux_state(NS(data=json.dumps({'input': source, 'since_ns': 300,
                                     'hold': {'cause': 'rc_stick', 'since_ns': 200}})))
    n._command((.12, 0., 0.))
    plain._command((.12, 0., 0.))
    assert n.sent[-1] == pytest.approx(plain.sent[-1]) and any(n.sent[-1])


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
