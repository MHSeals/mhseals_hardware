from mhseals_hardware.manual_state import ThrusterKeys


class Queue:
    def __init__(self, keys):
        self.keys = list(keys)

    def read(self, timeout=None):
        return self.keys.pop(0) if self.keys else None


def test_digits_pick_a_thruster_and_everything_else_passes():
    keys = ThrusterKeys(Queue(['press:3', 'release:3', 'press:w']))
    assert keys.read() == 'press:w' and keys.index == 2          # 3 is RR, then W drives it
    keys = ThrusterKeys(Queue(['2', 'repeat:2', 'w', '0', 'w']))
    assert keys.read() == 'w' and keys.index == 1
    assert keys.read() == 'w' and keys.index is None              # 0 is normal driving again


def test_v_toggles_speed_mode_and_clears_a_picked_thruster():
    keys = ThrusterKeys(Queue(['press:2', 'press:v', 'release:v', 'press:w', 'v', 'w']))
    assert keys.read() == 'press:w' and keys.velocity and keys.index is None
    assert keys.read() == 'w' and not keys.velocity


def test_g_releases_to_autonomy_until_space_or_another_mode():
    keys = ThrusterKeys(Queue(['press:2', 'press:g', 'release:g', 'press:w']))
    assert keys.read() == 'press:w' and keys.released and keys.index is None   # W passes; run_manual takes back on it
    keys = ThrusterKeys(Queue(['g', 'press:space']))
    assert keys.read() == 'press:space' and not keys.released                  # Space takes the boat back
    keys = ThrusterKeys(Queue(['g', 'v', 'w', 'g', 'g', '1', 'w']))
    assert keys.read() == 'w' and keys.velocity and not keys.released          # V leaves release for speed mode
    assert keys.read() == 'w' and keys.index == 0 and not keys.released        # G twice, then 1: thruster test
