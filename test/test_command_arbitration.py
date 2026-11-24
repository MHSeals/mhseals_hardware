import pytest

from mhseals_hardware.command_arbitration import Arbitration, INIT_KEYS, mode_from_pwm, mux_section
from mhseals_hardware.configuration import load_defaults



MODES = (0, 0, 0, 4, 0, 15)


def section(**changes):
    return {**mux_section(load_defaults()), **changes}


def arbitration(**changes):
    return Arbitration(**section(**changes))


LOW, MID, HIGH = 1100, 1550, 1900   # MANUAL, HOLD, GUIDED on the mode channel
PUSH = (1500, 1800, 1500, 1500)     # sway stick past the takeover


def frame(mode=HIGH, sticks=(1500, 1500, 1500, 1500), arm=1900):
    """Eight channels: sticks 1-4, ch6 stop switch, ch8 mode."""
    return [*sticks, 1500, arm, 1500, mode]


def boat():
    """A mux that has been running a while (no start-up ramp pending)."""
    a = arbitration()
    a._ramp_pending = False
    return a


def latch(a):
    """Take over from GUIDED with a held stick; return the time after it."""
    a.update_rc(frame(sticks=PUSH), 0)
    a.update_rc(frame(sticks=PUSH), 0.25)
    assert a.select(0.26)[0] == 'rc'
    return 0.26


def test_mode_bands_match_the_fcu():
    assert [mode_from_pwm(p, MODES) for p in (1100, 1300, 1400, 1550, 1700, 1900)] == [
        'MANUAL', 'MANUAL', 'MANUAL', 'HOLD', 'MANUAL', 'GUIDED']
    assert mode_from_pwm(1230, MODES) == 'MANUAL' and mode_from_pwm(1750, MODES) == 'GUIDED'


def test_mode5_band_decodes_manual_and_drives_rc():
    a = boat()
    a.update_rc(frame(mode=1700, sticks=(1500, 2000, 1500, 1500)), 0)
    assert a.select(0.01) == ('rc', (0.5, 0.0, 0.0))


def test_empty_short_or_wild_frames_are_invalid():
    a = boat()
    for t, ch in enumerate(([], [1500] * 3, frame(mode=2500))):
        a.update_rc(ch, t * 0.01)
        assert not a.valid(t * 0.01 + 0.005)


def test_priority_rc_manual_auto_idle():
    a = boat()
    assert a.select(0) == ('idle', (0, 0, 0))
    a.update_auto((1, 0, 0), 0)
    a.update_manual((0.2, 0, 0), 0)
    a.update_rc(frame(mode=LOW), 0)
    assert a.select(0.01)[0] == 'rc'
    a.update_rc(frame(mode=HIGH), 0.1)
    assert a.select(0.11) == ('manual', (0.2, 0, 0))
    a2 = boat()
    a2.update_auto((0.3, 0, 0), 0)
    assert a2.select(0.1) == ('auto', (0.3, 0, 0))
    assert a2.select(0.7)[0] == 'idle'


def test_keyboard_stopping_falls_through_to_auto_with_the_ramp_then_idle():
    """The owner's rule: a lost input goes neutral and nothing stays locked."""
    a = boat()
    for i in range(10):                               # keys held for 0.5 s
        a.update_manual((0.4, 0, 0), i * 0.05)
        assert a.select(i * 0.05) == ('manual', (0.4, 0, 0))
    # Ctrl-C with a key down: no more manual frames. A controller publishes effort.
    for t in (0.6, 0.8, 1.0):
        a.update_auto((1.0, 0, 0), t)
    assert a.select(0.5)[0] == 'manual'              # within command_timeout
    source, cmd = a.select(1.0)                       # manual stale: auto, ramping from zero
    assert source == 'auto' and cmd[0] < 0.5
    a.update_auto((1.0, 0, 0), 2.0)
    assert a.select(2.0) == ('auto', (1.0, 0, 0))     # fully ramped, no release step
    assert a.select(2.6) == ('idle', (0, 0, 0))       # auto stopped too: neutral
    assert a.state(2.6)['holds'] == []
    b = boat()
    b.update_manual((0.4, 0, 0), 0)
    assert b.select(0.6) == ('idle', (0, 0, 0))       # no other lane: neutral, no hold
    b.update_manual((0.2, 0, 0), 5.0)
    assert b.select(5.0) == ('manual', (0.2, 0, 0))   # the keys again drive at once


def test_rc_lost_falls_through_without_a_hold():
    a = boat()
    a.update_rc(frame(mode=LOW), 0)
    assert a.select(0.1)[0] == 'rc'
    assert a.select(0.5) == ('idle', (0, 0, 0))      # frame older than 0.3 s
    a.update_auto((0.3, 0, 0), 1.0)
    assert a.select(1.1)[0] == 'auto'


def test_switch_at_hold_holds_while_there_and_frees_when_moved():
    a = boat()
    a.update_rc(frame(mode=HIGH), 0)
    a.update_rc(frame(mode=MID), 0.1)
    a.update_auto((0.3, 0, 0), 0.1)
    assert a.select(0.11)[0] == 'hold'
    a.update_rc(frame(mode=HIGH), 0.2)
    a.update_auto((0.3, 0, 0), 0.2)
    assert a.select(0.21)[0] == 'auto'


def test_stop_switch_edge_holds_until_it_goes_back_up():
    a = boat()
    a.update_rc(frame(arm=1900), 0)
    a.update_rc(frame(arm=1100), 0.05)
    a.update_auto((0.3, 0, 0), 0.1)
    assert a.select(0.1)[0] == 'hold' and a.state(0.1)['cause'] == 'stop_channel'
    a.update_rc(frame(arm=1100), 0.15)                # still down: still held
    assert a.select(0.15)[0] == 'hold'
    a.update_rc(frame(arm=1900), 0.2)                 # back up: cleared
    a.update_auto((0.3, 0, 0), 0.2)
    assert a.select(0.2)[0] == 'auto'


def test_stick_takeover_then_release_falls_through():
    a = boat()
    a.update_rc(frame(sticks=PUSH), 0)
    assert a.select(0.1)[0] != 'rc'                  # not held long enough
    latch(a)
    for t in (0.3, 0.5, 0.7, 0.9):                   # sticks centered
        a.update_rc(frame(), t)
        a.update_auto((0.3, 0, 0), t)
    assert a.select(0.9)[0] == 'auto' and not a.latched


def test_latched_takeover_survives_a_gentle_offset():
    a = boat()
    latch(a)
    for t in (0.3, 0.5, 0.7, 0.9, 1.1):              # 150 us: inside takeover band
        a.update_rc(frame(sticks=(1650, 1500, 1500, 1500)), t)
    assert a.select(1.1)[0] == 'rc' and a.latched


def test_small_stick_movement_does_not_latch():
    a = boat()
    for t in (0, 0.1, 0.2, 0.3):
        a.update_rc(frame(sticks=(1600, 1500, 1500, 1500)), t)
    assert a.select(0.3)[0] != 'rc'


def test_rc_loss_drops_the_latch_until_a_new_takeover():
    a = boat()
    latch(a)
    assert a.select(1.0)[0] == 'idle' and not a.latched
    a.update_rc(frame(), 2.0)
    assert a.select(2.0)[0] == 'idle'


def test_hold_ramps_auto_from_zero_over_one_second():
    a = boat()
    a.update_rc(frame(mode=MID), 0)
    a.update_auto((1.0, 0, 0), 0)
    assert a.select(0.1)[0] == 'hold'
    a.update_rc(frame(mode=HIGH), 1.0)
    a.update_auto((1.0, 0, 0), 1.0)
    assert a.select(1.0) == ('auto', (0.0, 0, 0))
    a.update_rc(frame(mode=HIGH), 1.5)
    a.update_auto((1.0, 0, 0), 1.5)
    assert a.select(1.5)[1][0] == pytest.approx(0.5)


def test_idle_to_auto_is_not_ramped():
    a = boat()
    a.update_auto((1.0, 0, 0), 0)
    assert a.select(0) == ('auto', (1.0, 0, 0))


def test_state_reports_holds_and_decode():
    a = boat()
    a.update_rc(frame(mode=MID), 0)
    state = a.state(0.01)
    assert state['input'] == 'hold' and state['rc']['mode'] == 'HOLD'
    assert state['holds'][0]['cause'] == 'rc_switch'


@pytest.mark.parametrize('bad', [0, 65535, 2500])
def test_out_of_range_stick_neither_latches_nor_drives(bad):
    a = boat()
    for t in (0, 0.25):
        a.update_rc(frame(sticks=(1500, bad, 1500, 1500)), t)
    assert a.select(0.26)[1] == (0.0, 0.0, 0.0) and not a.latched


def test_zero_ramp_means_no_ramp():
    a = arbitration(ramp=0)
    a.update_rc(frame(mode=MID), 0)
    a.update_auto((0.4, 0.0, 0.0), 0.1)
    a.select(0.1)
    a.update_rc(frame(mode=HIGH), 0.2)
    a.update_auto((0.4, 0.0, 0.0), 0.3)
    assert a.select(0.31)[1] == (0.4, 0.0, 0.0)


def test_switch_left_at_hold_frees_on_dropout_and_holds_again_when_back():
    a = boat()
    a.update_rc(frame(mode=MID), 0)
    a.update_auto((0.4, 0.0, 0.0), 0)
    assert a.select(0.01)[0] == 'hold'
    a.update_auto((0.4, 0.0, 0.0), 5)
    assert a.select(5)[0] == 'auto'                   # RC gone: no stop is chosen any more
    a.update_rc(frame(mode=MID), 5.1)                 # back, still at HOLD
    assert a.select(5.11)[0] == 'hold'


@pytest.mark.parametrize('bad', [(float('nan'), 0, 0), (0, float('inf'), 0),
                                 (0, 0, float('-inf'))])
def test_non_finite_commands_never_reach_select(bad):
    a = boat()
    a.update_manual((0.1, 0, 0), 0)
    a.update_manual(bad, 0.1)
    assert a.select(0.2) == ('manual', (0.1, 0, 0))
    assert a.select(0.7)[0] == 'idle'       # keys timed out: the bad frame did not refresh it
    b = boat()
    b.update_auto((0.2, 0, 0), 0)
    b.update_auto(bad, 0.1)
    assert b.select(0.2)[1] == (0.2, 0, 0)
    assert b.select(0.7)[0] == 'idle'
    assert a.rejected == b.rejected == 1


def test_duplicate_mux_never_forwards_a_nonzero_frame():
    a = boat()
    a.set_duplicate(True, 0)
    for i in range(1, 60):
        t = i * 0.04
        a.update_auto((0.5, 0, 0), t)
        a.update_manual((0.3, 0, 0), t)
        a.update_rc(frame(mode=LOW, sticks=PUSH), t)
        assert a.select(t)[1] == (0.0, 0.0, 0.0)
    assert 'duplicate' in [h['cause'] for h in a.state(2.4)['holds']]


def test_duplicate_cleared_resumes_through_the_ramp():
    a = boat()
    a.set_duplicate(True, 0)
    a.update_auto((1, 0, 0), 1.0)
    a.set_duplicate(False, 1.0)
    assert a.select(1.1)[1][0] == 0.0     # the ramp starts from zero
    a.update_auto((1, 0, 0), 1.4)
    source, cmd = a.select(1.6)
    assert source == 'auto' and 0 < cmd[0] < 1


def test_bad_layout_is_rejected():
    for bad in (section(mode_ch=0), section(modes=[0, 4]), section(bogus=1), {'mode_ch': 7}):
        with pytest.raises((ValueError, TypeError)):
            Arbitration.from_config(bad)


def test_first_auto_after_start_ramps_in():
    a = arbitration(ramp=1.0)
    a.update_auto((1.0, 0.0, 0.0), 100.0)
    assert a.select(100.0) == ('auto', (0.0, 0.0, 0.0))
    assert a.select(100.5)[1][0] == pytest.approx(0.5)


STATE_FIELDS = {'input', 'since_ns', 'cause', 'hold', 'rc', 'manual', 'auto',
                'rejected', 'pipeline_ms', 'holds'}
RC_FIELDS = {'valid', 'mode', 'latched', 'stick', 'pwm_mode', 'rate_hz',
             'interval_p95_s', 'age_s', 'stop_edge'}


def test_state_has_the_authority_fields_and_wall_clock_times():
    a = arbitration()
    for i in range(5):
        a.update_rc([1500, 1500, 1500, 1500, 1500, 1800, 1500, 1100], i * 0.04)
        a.select(i * 0.04)            # the node selects on every frame
    state = a.state(0.2, wall_ns=10**18, pipeline_ms=0.8)
    assert STATE_FIELDS <= set(state) and RC_FIELDS <= set(state['rc'])
    assert state['input'] == 'rc' and state['cause'] == 'rc_switch'
    assert state['rc']['pwm_mode'] == 1100 and state['rc']['rate_hz'] == 25.0
    assert state['pipeline_ms'] == 0.8 and state['auto'] == {'age_s': None, 'active': False}
    assert state['since_ns'] == 10**18 - int(0.2e9)


def test_state_cause_names_the_hold():
    a = arbitration()
    a.update_rc([1500, 1500, 1500, 1500, 1500, 1800, 1500, 1500], 0.0)   # HOLD switch
    state = a.state(0.01, wall_ns=10**18)
    assert state['cause'] == 'rc_switch' and state['hold']['by'] == 'rc'
    assert state['holds'][0]['since_ns'] == 10**18 - int(0.01e9)


def test_yaml_section_equals_the_former_python_defaults():
    a = arbitration()
    assert (a.mode_ch, a.modes, a.stop_ch, a.rc_scale) == (8, MODES, 6, (0.5, 0.5, 0.3))
    assert (a.rc_max_age, a.command_timeout, a.stick_takeover) == (0.3, 0.5, 200)
    assert (a.stick_time, a.stick_release, a.ramp) == (0.2, 0.5, 1.0)
    assert set(mux_section(load_defaults())) == set(INIT_KEYS)
