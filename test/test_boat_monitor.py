import json
import math

from mhseals_hardware.boat_monitor import BoatMonitor


def test_receipt_rate_quality_and_stamp_freshness():
    clock = [0.0]
    monitor = BoatMonitor(clock=lambda: clock[0])
    assert monitor.snapshot()['sensors'] == {}
    for index in range(10):
        clock[0] = index / 10
        monitor.observe('IMU', stamp_age=.05)
    imu = monitor.snapshot()['sensors']['IMU']
    assert imu['status'] == 'LIVE'
    assert math.isclose(imu['hz'], 10)
    monitor.observe('GPS', issue='no fix', stamp_age=0)
    assert monitor.snapshot()['sensors']['GPS']['status'] == 'WARN'
    monitor.observe('future', stamp_age=-2)
    assert monitor.snapshot()['sensors']['future']['status'] == 'WARN'
    clock[0] += 3
    assert monitor.snapshot()['sensors']['IMU']['status'] == 'STALE'


def test_tf_connectivity_static_age_conflict_cycle_and_recovery():
    clock = [0.0]
    monitor = BoatMonitor(clock=lambda: clock[0])
    monitor.transform('map', 'odom', static=True, authority='global')
    monitor.transform('odom', 'base_link', authority='local')
    assert all(row['status'] == 'LIVE' for row in monitor.snapshot()['tf'])
    monitor.transform('odom', 'base_link', authority='duplicate')
    assert monitor.snapshot()['tf'][1]['status'] == 'WARN'
    clock[0] = 3
    assert monitor.snapshot()['tf'][0]['status'] == 'LIVE'
    assert monitor.snapshot()['tf'][1]['status'] == 'MISSING'
    monitor.transform('odom', 'base_link', authority='local')
    assert monitor.snapshot()['tf'][1]['status'] == 'LIVE'
    monitor.transform('base_link', 'odom', authority='bad_cycle')
    assert monitor.snapshot()['tf'][1]['status'] == 'WARN'


def test_trial_reports_drift_adjusted_sign_not_false_calibration_pass():
    clock = [0.0]
    monitor = BoatMonitor(clock=lambda: clock[0])
    for tick in range(60):
        clock[0] = tick / 10
        monitor.observe('odometry', dict(vx=.1 if tick < 20 else .3,
                                        vy=0., wz=0., frame='odom', child='base_link'))
    result = monitor.summarize_trial('surge', 1, 0, 2, 5.9)
    assert result['verdict'] == 'SIGN MATCH'
    assert math.isclose(result['response'], .2)
    assert monitor.summarize_trial('surge', -1, 0, 2, 5.9)['verdict'] == 'SIGN MISMATCH'
    assert monitor.summarize_trial('sway', 1, 0, 2, 5.9)['verdict'] == 'LOW RESPONSE'
    assert monitor.summarize_trial('surge', 1, 20, 21, 22)['verdict'] == 'INCONCLUSIVE'


def test_bad_numeric_data_is_exportable_and_never_scores_a_trial():
    monitor = BoatMonitor(clock=lambda: 1)
    monitor.observe('odometry', {'vx': float('nan')})
    snapshot = monitor.snapshot()
    assert snapshot['sensors']['odometry']['status'] == 'WARN'
    json.dumps(snapshot, allow_nan=False)


def test_history_and_events_are_bounded():
    monitor = BoatMonitor(clock=lambda: 1)
    for index in range(13000):
        monitor.observe('odometry', {'vx': index})
        monitor.event('event')
    assert len(monitor.history) == 12000
    assert len(monitor.events) == 12
    assert len(monitor.sensors['odometry']['times']) == 300
