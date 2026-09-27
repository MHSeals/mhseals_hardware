import io

import pytest

pytest.importorskip('rich')
from rich.console import Console
from mhseals_hardware.boat_dashboard import render_dashboard, VIEWS
from mhseals_hardware.boat_monitor import BoatMonitor
from mhseals_hardware.thruster_mixer import THRUSTER_MIXER


@pytest.mark.parametrize('width,height', [(80, 24), (120, 40)])
@pytest.mark.parametrize('view', VIEWS)
def test_all_views_keep_safety_controls_visible(width, height, view):
    stream = io.StringIO()
    console = Console(file=stream, width=width, height=height, color_system=None)
    monitor = BoatMonitor()
    monitor.observe('odometry', {'vx': .2, 'vy': 0., 'wz': -.1, 'child': 'base_link'})
    context = dict(view=view, specs=[('odometry', '/odom/local', True),
                                   ('GPS', '/gps/fix', True), ('IMU', '/imu/raw', True)],
                   channel_map=[2, 1, 4, 3], matrix=THRUSTER_MIXER,
                   chips=['/sys/pwm/one', '/sys/pwm/two', '/sys/pwm/three', '/sys/pwm/four'],
                   channels=[0]*4, host='test-boat', domain='42', notice='Ready')
    console.print(render_dashboard(monitor.snapshot(), context, width, height))
    text = stream.getvalue()
    assert 'DISARMED' in text and 'Space STOP' in text and 'Q quit' in text
    assert len(text.splitlines()) <= height
    assert all(len(line) <= width for line in text.splitlines())
    if view == 'overview':
        assert 'Measured' in text and 'BOW' in text


def test_manual_view_does_not_advertise_inactive_dashboard_keys():
    stream = io.StringIO()
    console = Console(file=stream, width=100, height=32, color_system=None)
    context = dict(manual=True, specs=[], matrix=THRUSTER_MIXER)
    console.print(render_dashboard(BoatMonitor().snapshot(), context))
    assert 'M/X/Esc return' in stream.getvalue()
    assert 'H arm/disarm' not in stream.getvalue()
