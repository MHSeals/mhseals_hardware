from mhseals_hardware import com_trim
from mhseals_hardware.thruster_mixer import THRUSTER_MIXER, mix_thrusters, validate_decoupling


class Queue:
    def __init__(self, keys):
        self.keys = list(keys)

    def read(self, timeout=None):
        return self.keys.pop(0) if self.keys else None


def yaw_of(efforts):
    """The mixer's yaw wrench of four efforts (thrust e|e| times each thruster's yaw gain)."""
    return sum(e * abs(e) * row[2] for e, row in zip(efforts, THRUSTER_MIXER))


def test_a_starboard_or_forward_marker_adds_ccw_yaw_to_surge_or_sway_both_ways():
    trim = com_trim.Trim()
    for key in 'llli':
        trim.key(key)
    c = trim.matrix()
    assert (trim.right, trim.fwd) == (3, 1)
    assert c[com_trim.YAW_SURGE] == 0.03 and c[com_trim.YAW_SWAY] == 0.01
    assert c[:6] == com_trim.IDENTITY[:6] and c[8] == 1.0      # everything else is identity
    rows = validate_decoupling(c)
    # CoM to starboard: forward thrust yaws CW, so the trim adds CCW (+) yaw; reverse yaws CCW, the trim CW
    assert yaw_of(mix_thrusters(0.5, 0, 0, decoupling=rows)) > 0
    assert yaw_of(mix_thrusters(-0.5, 0, 0, decoupling=rows, decoupling_negative=rows)) < 0
    # CoM forward: port sway yaws CW, so the trim adds CCW yaw
    assert yaw_of(mix_thrusters(0, 0.5, 0, decoupling=rows)) > 0


def test_keys_step_once_per_press_clamp_and_centre():
    trim = com_trim.Trim(limit=2)
    keys = com_trim.TrimKeys(Queue(['press:t', 'press:l', 'repeat:l', 'repeat:l', 'release:l', 'L', 'l', 'l',
                                    'press:w']), trim)
    assert keys.read() == 'press:w'                 # T opens the layer; trim keys are eaten, the rest passes
    assert trim.right == 2 and trim.touched and trim.dirty       # repeats dropped, clamped at the limit
    trim.key('c')
    assert trim.matrix() == com_trim.IDENTITY


def test_the_layer_starts_collapsed_and_t_toggles_it():
    trim = com_trim.Trim()
    keys = com_trim.TrimKeys(Queue(['press:l', 'press:t', 'press:l', 'release:t', 't', 'press:j']), trim)
    assert keys.read() == 'press:l' and trim.right == 0     # hidden: L passes through and trims nothing
    assert keys.read() == 'press:j' and trim.right == 1     # T opens, L steps, T (plain) hides, J passes
    assert not trim.shown and trim.lines().count('\n') == 0   # collapsed to one line ...
    assert '+1 stbd' in trim.lines()                         # ... that still shows the active trim
    trim.shown = True
    assert 'bow' in trim.lines() and 'T hide' in trim.lines()


def test_the_result_sets_c_and_c_minus_for_astro_apply(tmp_path):
    trim = com_trim.Trim()
    trim.key('j')
    data = com_trim.result(trim, before=[1.0, 0, 0, 0, 1.0, 0, 0.2, 0, 1.0])
    assert data['status'] == 'done' and data['test'] == 'com-trim'
    assert [c['key'] for c in data['changes']] == ['decoupling', 'decoupling_negative']
    assert all(c['value'][com_trim.YAW_SURGE] == -0.01 for c in data['changes'])
    path = com_trim.save(data, tmp_path)
    assert path.parent.name == 'results' and path.parent.parent.parent == tmp_path
    trim.shown = True
    assert 'C before' in trim.lines(data['before']) and '●' in trim.lines()
