import json

from mhseals_hardware import fcu_params


class FakeFcu:
    """An FCU that stores integers and refuses one name."""

    def __init__(self, values, refuse=()):
        self.values, self.refuse = dict(values), set(refuse)

    def get(self, names):
        return {n: self.values.get(n) for n in names}

    def set(self, values, current):
        for name, value in values.items():
            if name not in self.refuse:
                self.values[name] = int(value)
        return {n: 'refused' for n in values if n in self.refuse}


def test_apply_backs_up_reads_back_and_undo_restores(tmp_path, capsys):
    history = tmp_path / 'history.jsonl'
    fcu = FakeFcu({'EK3_SRC1_YAW': 1, 'OTHER': 5})
    original = fcu_params.HISTORY
    fcu_params.HISTORY = history
    try:
        assert fcu_params.write(fcu, {'EK3_SRC1_YAW': 8, 'OTHER': 5}, 'no-compass', None) == 0
        assert fcu.values['EK3_SRC1_YAW'] == 8
        entries = fcu_params.history(history)
        assert entries[-1]['values'] == {'EK3_SRC1_YAW': [1, 8]}       # OTHER was already set: not recorded
        old = {n: o for n, (o, _) in entries[-1]['values'].items()}
        assert fcu_params.write(fcu, old, 'undo no-compass', None) == 0
        assert fcu.values['EK3_SRC1_YAW'] == 1
    finally:
        fcu_params.HISTORY = original


def test_a_refused_or_missing_parameter_is_reported(tmp_path, capsys):
    fcu_params_history = tmp_path / 'h.jsonl'
    original = fcu_params.HISTORY
    fcu_params.HISTORY = fcu_params_history
    try:
        fcu = FakeFcu({'A': 1}, refuse={'A'})
        assert fcu_params.write(fcu, {'A': 2}, 'x', None) == 1
        assert 'NOT SET' in capsys.readouterr().out
        assert fcu_params.write(FakeFcu({}), {'MISSING': 1}, 'x', None) == 1
        assert not fcu_params_history.exists()
    finally:
        fcu_params.HISTORY = original


def test_lists_load_and_the_user_file_replaces_by_name(tmp_path):
    package = tmp_path / 'p.yaml'
    package.write_text('lists:\n  a: {description: one, params: {X: 1}}\n  b: {params: {Y: 2}}\n')
    user = tmp_path / 'u.yaml'
    user.write_text('lists:\n  a: {description: mine, params: {X: 3}}\n')
    lists = fcu_params.load_lists([package, user])
    assert lists['a']['params'] == {'X': 3} and lists['b']['params'] == {'Y': 2}
    assert set(fcu_params.load_lists([fcu_params.package_file()])) >= {'no-compass', 'compass'}
    json.dumps(lists)


def test_magcal_sends_the_heading_and_records_the_old_offsets(tmp_path, capsys):
    class MagFcu(FakeFcu):
        sent = None

        def command(self, number, params):
            self.sent = (number, params)
            self.values.update(COMPASS_OFS_X=12, COMPASS_OFS_Y=-30)
            return True, 0

        def pull(self):
            pass
    original = fcu_params.HISTORY
    fcu_params.HISTORY = tmp_path / 'h.jsonl'
    try:
        fcu = MagFcu({'COMPASS_OFS_X': 0, 'COMPASS_OFS_Y': 0, 'COMPASS_OFS_Z': 5})
        assert fcu_params.magcal(fcu, 90.0) == 0
        assert fcu.sent == (42006, [90.0, 0, 0, 0])
        entry = fcu_params.history()[-1]
        assert entry['values'] == {'COMPASS_OFS_X': [0, 12], 'COMPASS_OFS_Y': [0, -30]}
    finally:
        fcu_params.HISTORY = original
