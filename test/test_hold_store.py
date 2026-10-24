"""A restarted mux re-adopts its holds; a bad file starts clean."""

import pytest

from mhseals_hardware import hold_store
from mhseals_hardware.command_arbitration import Arbitration
from mhseals_hardware.configuration import load_defaults


def arbitration(**changes):
    return Arbitration(**{**load_defaults()['command_mux'], **changes})


def restarted(path, now):
    """What a fresh mux process does at start."""
    a = arbitration(ramp=1.0)
    a.adopt_holds(hold_store.load(path), now)
    return a


def test_keys_hold_survives_restart(tmp_path):
    path = tmp_path / 'run' / 'command_mux.json'
    old = arbitration()
    old.request('hold', 'keys', 'keys', 1.0)
    hold_store.save(path, old.holds)
    new = restarted(path, 50.0)
    new.update_auto((1.0, 0.0, 0.0), 50.0)
    assert new.select(50.0) == ('hold', (0.0, 0.0, 0.0))
    assert new.holds['keys']['by'] == 'keys'


def test_no_holds_restart_ramps_auto(tmp_path):
    path = tmp_path / 'command_mux.json'
    hold_store.save(path, {})
    new = restarted(path, 50.0)
    new.update_auto((1.0, 0.0, 0.0), 50.0)
    assert new.select(50.0)[1][0] == 0.0
    assert new.select(50.5)[1][0] == pytest.approx(0.5)


@pytest.mark.parametrize('text', [None, '', '[{"cause": "keys"', '{"a": 1}', '[1]'])
def test_torn_or_missing_file_starts_clean(tmp_path, text):
    path = tmp_path / 'command_mux.json'
    if text is not None:
        path.write_text(text)
    assert restarted(path, 1.0).holds == {}


def test_file_holds_only_a_token_hash(tmp_path):
    path = tmp_path / 'command_mux.json'
    a = arbitration()
    a.request('hold', 'apply', 'config', 0, token='s3cret')
    hold_store.save(path, a.holds)
    assert 's3cret' not in path.read_text()
    assert not list(tmp_path.glob('*.tmp'))


@pytest.mark.parametrize('cause', ['rc_stick', 'rc_stop', 'rc_lost', 'rc_switch'])
def test_adopted_rc_hold_survives_the_first_guided_frame(cause):
    a = arbitration(ramp=1.0)
    a.adopt_holds({cause: {'cause': cause, 'by': 'rc', 'since': 1.0, 'token': ''}}, 50.0)
    a.update_auto((1.0, 0.0, 0.0), 50.0)
    a.update_rc([1500, 1500, 1500, 1500, 1500, 1900, 1500, 1900], 50.0)   # GUIDED
    assert a.select(50.0) == ('hold', (0.0, 0.0, 0.0))
    assert cause in a.holds
