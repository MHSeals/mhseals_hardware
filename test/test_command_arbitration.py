from mhseals_hardware.command_arbitration import Arbitration, mode_from_pwm

LOW, MID, HIGH = 1100, 1550, 1900   # MANUAL, HOLD, GUIDED on the mode channel


def frame(mode=HIGH, sticks=(1500, 1500, 1500, 1500), arm=1900):
    """Eight channels: sticks 1-4, ch6 stop switch, ch8 mode."""
    return [*sticks, 1500, arm, 1500, mode]


def boat():
    return Arbitration()


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


def test_state_reports_holds_and_decode():
    a = boat()
    a.update_rc(frame(mode=MID), 0)
    state = a.state(0.01)
    assert state['input'] == 'hold' and state['rc']['mode'] == 'HOLD'
    assert state['holds'][0]['cause'] == 'rc_switch'
