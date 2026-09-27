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
