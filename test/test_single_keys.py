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
