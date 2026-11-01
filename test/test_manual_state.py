from mhseals_hardware.manual_state import ManualState


def test_initial_repeat_gap_and_fast_stop():
    state = ManualState()
    assert state.update('w', 0) == (.25, 0, 0)
    assert state.update(None, .3) == (.25, 0, 0)
    assert state.update('w', .5) == (.25, 0, 0)
    assert state.update('w', .55) == (.25, 0, 0)
    assert state.update(None, .71) == (0, 0, 0)


def test_release_and_space_stop_immediately():
    for key in ('release:w', 'space'):
        state = ManualState()
        state.update('w', 0)
        assert state.update(key, .01) == (0, 0, 0)


def test_speed_does_not_extend_deadman():
    state = ManualState()
    state.update('w', 0)
    assert state.update('+', .1) == (.3, 0, 0)
    assert state.update('+', .7) == (0, 0, 0)
    for _ in range(100):
        state.update('+', 1)
    assert state.amplitude == 1
    for _ in range(100):
        state.update('-', 1)
    assert state.amplitude == 0


def test_unrelated_release_does_not_stop_current_direction():
    state = ManualState()
    state.update('w', 0)
    state.update('s', .1)
    assert state.update('release:w', .11) == (-.25, 0, 0)


def test_yaw_scale_is_validated_and_applied():
    import pytest
    assert ManualState(yaw_scale=1.0).update('left', 0) == (0, 0, .25)
    for bad in (0, 1.5, float('nan')):
        with pytest.raises(ValueError):
            ManualState(yaw_scale=bad)


def test_non_motion_releases_do_not_keep_a_held_key_alive():
    state = ManualState()
    state.update('press:w', 0)
    state.update('press:a', .1)
    state.update('release:a', .2)  # w's own release is lost from here on
    for t in (1.0, 2.0, 3.0):
        state.update('release:+', t)
        state.update('release:d', t)  # never held
    assert state.update(None, 3.5) == (0, 0, 0)


def test_event_mode_alias_is_gone():
    assert not hasattr(ManualState(), 'event_mode')


def test_a_reported_hold_survives_a_link_stall_but_input_loss_still_stops():
    state = ManualState()
    state.update('press:w', 0)
    state.update('repeat:w', .5)
    assert state.update(None, .8) == (0, 0, 0)             # 0.3 s stall: the watchdog stops it
    assert state.update('repeat:w', .9) == (.25, 0, 0)     # no release seen: its repeat re-arms it
    assert state.update(None, 1.06) == (0, 0, 0)           # and lost input still stops within the timeout


def test_release_space_or_a_long_gap_keep_a_stalled_key_stopped():
    for stop in ('release:w', 'space', 'focus-out'):
        state = ManualState()
        state.update('press:w', 0)
        state.update(None, .7)                              # stalled
        state.update(stop, .8)
        assert state.update('repeat:w', .9) == (0, 0, 0)
    state = ManualState()
    state.update('press:w', 0)
    state.update(None, .7)
    assert state.update('repeat:w', 3.0) == (0, 0, 0)      # past REARM_S: press again


def test_plain_terminal_repeats_never_rearm_a_stopped_key():
    state = ManualState()
    state.update('w', 0)
    state.update(None, .7)
    assert state.update('w', .8) == (.25, 0, 0) and state.single == 'w'   # a plain key is a new press
    state = ManualState()
    state.update('w', 0)
    state.update(None, .7)
    assert state.update('repeat:w', .8) == (0, 0, 0)      # it never reported events: no re-arm


def test_drain_reads_a_whole_stale_burst_before_publishing():
    from mhseals_hardware.manual_state import drain

    class Keys:
        def __init__(self, queued):
            self.queued = list(queued)

        def read(self, timeout=None):
            return self.queued.pop(0) if self.queued else None
    # A link stall delivers a held key's repeats and its release together, more than 64 events.
    keys = Keys(['repeat:w'] * 200 + ['release:w'])
    assert drain(keys, ManualState(), 'press:w', clock=lambda: 1.0) == (0, 0, 0)
    assert drain(Keys(['release:w']), ManualState(), 'x', clock=lambda: 1.0) is None
    ticks = iter(range(100))     # past DRAIN_S after one event: the rest stays queued
    keys = Keys(['release:w'])
    assert drain(keys, ManualState(), 'press:w', clock=lambda: next(ticks)) == (.25, 0, 0)
    assert keys.queued == ['release:w']


class ClockedKeys:
    """Queued keys on a fake clock: every clock read costs ``cost`` s."""

    def __init__(self, queued, cost=1e-4):
        self.queued, self.now, self.cost = list(queued), 0.0, cost

    def clock(self):
        self.now += self.cost
        return self.now

    def read(self, timeout=None):
        return self.queued.pop(0) if self.queued else None


def test_drain_is_bounded_from_the_last_publish_not_from_the_batch_start():
    from mhseals_hardware.manual_state import drain
    keys = ClockedKeys(['repeat:w'] * 5000)
    keys.now = 0.05                  # the first read waited its 50 ms timeout after the publish at 0
    drain(keys, ManualState(), 'press:w', keys.clock, since=0.0)
    assert 0.09 < keys.now < 0.11    # ~DRAIN_S after the publish, not 50 ms wait + 100 ms
    assert keys.queued               # cut by time: the rest stays queued
    keys = ClockedKeys(['repeat:w'] * 5000)
    keys.now = 0.05
    drain(keys, ManualState(), 'press:w', keys.clock)
    assert keys.now > 0.149          # without since: the old bound, from the batch start


def test_drain_after_a_stall_past_the_budget_still_reads_the_stale_burst_whole():
    from mhseals_hardware.manual_state import drain
    keys = ClockedKeys(['repeat:w'] * 200 + ['release:w'], cost=1e-6)
    keys.now = 30.0                  # the link stalled 30 s after the last publish
    assert drain(keys, ManualState(), 'press:w', keys.clock, since=0.0) == (0, 0, 0)
