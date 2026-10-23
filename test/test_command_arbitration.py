import pytest

from mhseals_hardware.command_arbitration import Arbitration, mode_from_pwm

LOW, MID, HIGH = 1100, 1550, 1900   # MANUAL, HOLD, GUIDED on the mode channel
PUSH = (1500, 1800, 1500, 1500)     # sway stick past the takeover


def frame(mode=HIGH, sticks=(1500, 1500, 1500, 1500), arm=1900):
    """Eight channels: sticks 1-4, ch6 stop switch, ch8 mode."""
    return [*sticks, 1500, arm, 1500, mode]


def boat():
    return Arbitration()


def latch(a):
    """Take over from GUIDED with a held stick; return the time after it."""
    a.update_rc(frame(sticks=PUSH), 0)
    a.update_rc(frame(sticks=PUSH), 0.25)
    assert a.select(0.26)[0] == 'rc'
    return 0.26


def test_mode_bands_match_the_fcu():
    assert [mode_from_pwm(p) for p in (1100, 1300, 1400, 1550, 1700, 1900)] == [
        'MANUAL', 'MANUAL', 'MANUAL', 'HOLD', 'MANUAL', 'GUIDED']
    assert mode_from_pwm(1230) == 'MANUAL' and mode_from_pwm(1750) == 'GUIDED'


def test_mode5_band_decodes_manual_and_drives_rc():
    a = boat()
    a.update_rc(frame(mode=1700, sticks=(1500, 2000, 1500, 1500)), 0)
    assert a.select(0.01) == ('rc', (0.5, 0.0, 0.0))


def test_empty_short_or_wild_frames_are_invalid():
    a = boat()
    for t, ch in enumerate(([], [1500] * 3, frame(mode=2500))):
        a.update_rc(ch, t * 0.01)
        assert not a.valid(t * 0.01 + 0.005)


def test_priority_rc_manual_hold_auto_idle():
    a = boat()
    assert a.select(0) == ('idle', (0, 0, 0))
    a.update_auto((1, 0, 0), 0)
    a.update_manual((0.2, 0, 0), 0)
    a.update_rc(frame(mode=LOW), 0)
    assert a.select(0.01)[0] == 'rc'
    a.update_rc(frame(mode=HIGH), 0.1)
    assert a.select(0.11) == ('manual', (0.2, 0, 0))
    a.request('hold', 'space', 'tui', 0.12)
    assert a.select(0.13)[0] == 'manual'            # manual outranks hold
    assert a.select(1.1)[0] == 'hold'               # keys ended: still held
    a2 = boat()
    a2.update_auto((0.3, 0, 0), 0)
    assert a2.select(0.1) == ('auto', (0.3, 0, 0))
    assert a2.select(0.7)[0] == 'idle'


def test_rc_lost_while_active_holds():
    a = boat()
    a.update_rc(frame(mode=LOW), 0)
    assert a.select(0.1)[0] == 'rc'
    assert a.select(0.5) == ('hold', (0, 0, 0))     # frame older than 0.3 s
    assert 'rc_lost' in a.holds


def test_stale_rc_that_was_never_active_does_not_hold():
    a = boat()
    a.update_rc(frame(mode=HIGH), 0)
    assert a.select(1.0)[0] == 'idle' and not a.holds


def test_switch_to_hold_sets_a_hold_and_guided_clears_it():
    a = boat()
    a.update_rc(frame(mode=HIGH), 0)
    a.update_rc(frame(mode=MID), 0.1)
    a.update_auto((0.3, 0, 0), 0.1)
    assert a.select(0.11)[0] == 'hold'
    a.update_rc(frame(mode=HIGH), 0.2)
    a.update_auto((0.3, 0, 0), 0.2)
    assert a.select(0.21)[0] == 'auto'


def test_guided_never_clears_a_keys_hold():
    a = boat()
    a.update_rc(frame(mode=MID), 0)
    a.request('hold', 'keys', 'tui', 0.01)
    a.update_rc(frame(mode=HIGH), 0.1)
    assert list(a.holds) == ['keys']


def test_keyboard_ending_sets_a_keys_hold():
    a = boat()
    a.update_manual((0.2, 0, 0), 0)
    assert a.select(0.1)[0] == 'manual'
    assert a.select(0.6)[0] == 'hold' and a.holds['keys']['by'] == 'keys'


def test_stop_switch_falling_edge_holds():
    a = boat()
    a.update_rc(frame(arm=1900), 0)
    a.update_rc(frame(arm=1100), 0.05)
    assert 'rc_stop' in a.holds
    a.update_rc(frame(mode=MID), 0.15)
    a.update_rc(frame(mode=HIGH), 0.2)               # GUIDED clears RC causes
    assert not a.holds


def test_stick_latch_takes_over_then_holds_on_release():
    a = boat()
    a.update_rc(frame(sticks=PUSH), 0)
    assert a.select(0.1)[0] != 'rc'                  # not held long enough
    latch(a)
    for t in (0.3, 0.5, 0.7, 0.9):                   # sticks centered
        a.update_rc(frame(), t)
    assert a.select(0.9)[0] == 'hold' and 'rc_stick' in a.holds


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
    assert a.select(1.0)[0] == 'hold' and not a.latched
    a.update_rc(frame(), 2.0)
    assert a.select(2.0)[0] == 'hold'


def test_release_refused_at_hold_or_manual_switch():
    a = boat()
    a.update_rc(frame(mode=MID), 0)
    ok, why = a.request('release', 'keys', 'u', 0.01)
    assert not ok and 'HOLD' in why
    a.update_rc(frame(mode=LOW), 0.1)
    assert not a.request('release', 'keys', 'u', 0.11)[0]


def test_u_release_clears_every_cause_in_guided():
    a = boat()
    a.request('hold', 'keys', 'tui', 0)
    a.request('hold', 'space', 'tui', 0)
    a.update_rc(frame(mode=HIGH), 0.1)
    assert a.request('release', None, 'tui', 0.11) == (True, '')
    assert not a.holds


def test_token_release_clears_only_its_cause():
    a = boat()
    a.request('hold', 'apply', 'config', 0, token='t1')
    a.request('hold', 'keys', 'tui', 0)
    assert not a.request('release', 'apply', 'x', 0.1, token='bad')[0]
    assert a.request('release', 'apply', 'config', 0.1, token='t1')[0]
    assert list(a.holds) == ['keys']


def test_release_ramps_auto_from_zero_over_one_second():
    a = boat()
    a.request('hold', 'space', 'tui', 0)
    a.update_auto((1.0, 0, 0), 0)
    assert a.select(0.1)[0] == 'hold'
    a.request('release', None, 'tui', 1.0)
    a.update_auto((1.0, 0, 0), 1.0)
    assert a.select(1.0) == ('auto', (0.0, 0, 0))
    a.update_auto((1.0, 0, 0), 1.5)
    assert a.select(1.5)[1][0] == pytest.approx(0.5)
    a.update_auto((1.0, 0, 0), 2.1)
    assert a.select(2.1)[1][0] == 1.0


def test_hold_between_selects_still_ramps():
    a = boat()
    a.update_auto((1, 0, 0), 0)
    assert a.select(0)[0] == 'auto'
    a.request('hold', 'space', 'tui', 0.01)
    a.request('release', None, 'tui', 0.1)
    a.update_auto((1, 0, 0), 0.1)
    assert a.select(0.1) == ('auto', (0.0, 0, 0))


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
    a = Arbitration(ramp=0)
    a.update_rc(frame(mode=MID), 0)
    a.update_auto((0.4, 0.0, 0.0), 0.1)
    a.select(0.1)
    a.update_rc(frame(mode=HIGH), 0.2)
    a.update_auto((0.4, 0.0, 0.0), 0.3)
    assert a.select(0.31)[1] == (0.4, 0.0, 0.0)


def test_switch_left_at_hold_holds_again_after_dropout_and_release():
    a = boat()
    a.update_rc(frame(mode=MID), 0)
    a.update_auto((0.4, 0.0, 0.0), 0)
    assert a.select(0.01)[0] == 'hold'
    assert a.request('release', None, 'tui', 5)[0]     # RC stale: allowed
    a.update_rc(frame(mode=MID), 5.1)                  # back, still at HOLD
    a.update_auto((0.4, 0.0, 0.0), 5.1)
    assert a.select(5.11)[0] == 'hold'


@pytest.mark.parametrize('bad', [(float('nan'), 0, 0), (0, float('inf'), 0),
                                 (0, 0, float('-inf'))])
def test_non_finite_commands_never_reach_select(bad):
    a = boat()
    a.update_manual((0.1, 0, 0), 0)
    a.update_manual(bad, 0.1)
    assert a.select(0.2) == ('manual', (0.1, 0, 0))
    assert a.select(0.7)[0] == 'hold'       # keys timed out: the bad frame did not refresh it
    b = boat()
    b.update_auto((0.2, 0, 0), 0)
    b.update_auto(bad, 0.1)
    assert b.select(0.2)[1] == (0.2, 0, 0)
    assert b.select(0.7)[0] == 'idle'
    assert a.rejected == b.rejected == 1


def test_state_never_publishes_a_hold_token():
    a = Arbitration()
    a.request('hold', 'apply', 'config_revert', 0, token='s3cret')
    state = a.state(0.1)
    assert 's3cret' not in repr(state)
    assert state['holds'][0]['has_token'] is True
    assert 'token' not in state['holds'][0]


def test_releases_record_who_and_whether_tokenless():
    a = Arbitration()
    a.request('hold', 'apply', 'config', 0, token='t1')
    a.request('release', 'apply', 'config', 0.1, token='t1')
    assert a.state(0.2)['last_release'] == {
        'by': 'config', 'cause': 'apply', 'tokenless': False, 'cleared': 1}
    a.request('hold', 'keys', 'tui', 0.3)
    a.request('release', '', 'tui', 0.4)
    assert a.state(0.5)['last_release']['tokenless'] is True
    assert a.state(0.5)['last_release']['by'] == 'tui'


def test_duplicate_mux_never_forwards_a_nonzero_frame():
    held, other = boat(), boat()
    held.request('hold', 'space', 'tui', 0)
    held.set_duplicate(True, 0)
    other.set_duplicate(True, 0)
    for i in range(1, 60):
        t = i * 0.04
        held.update_auto((0.5, 0, 0), t)
        other.update_auto((0.5, 0, 0), t)
        other.update_manual((0.3, 0, 0), t)
        for a in (held, other):
            assert a.select(t)[1] == (0.0, 0.0, 0.0)
    assert held.request('release', '', 'tui', 3.0)[0]
    assert held.select(3.0)[1] == (0.0, 0.0, 0.0)
    assert 'duplicate' in [h['cause'] for h in held.state(3.0)['holds']]


def test_duplicate_cleared_resumes_through_the_ramp():
    a = boat()
    a.set_duplicate(True, 0)
    a.update_auto((1, 0, 0), 1.0)
    a.set_duplicate(False, 1.0)
    assert a.select(1.1)[1][0] == 0.0     # the ramp starts from zero
    a.update_auto((1, 0, 0), 1.4)
    source, cmd = a.select(1.6)
    assert source == 'auto' and 0 < cmd[0] < 1
