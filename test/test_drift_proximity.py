"""The drift observer doesn't grow away from a close obstacle (D1; /tmp/d1proto/proto.py).

Docked creep for 60 s against a dock on the world -y side, drift_limit 0.3, 10 seeds;
wound means |drift| > 0.05 m/s. Odometry noise as the prototype's: a 5 mm/s bias plus
2 cm/s white, or colored (first order) at tau 0.5, 3 and 10 s.
"""
import math

import numpy
import pytest

from mhseals_hardware.velocity_law import Plant, VelocityLaw, rotate
from test_velocity_law import Boat, Docked, config

DT, SEEDS, WOUND = 0.05, range(10), 0.05
DOCK = (0.0, -0.1)  # body frame: 10 cm off the starboard side (the boat heads +x)
NOISE = {'colored': (3.0, 0.025), 'colored10': (10.0, 0.03), 'colored.5': (0.5, 0.03)}


def noise(kind, rng):
    state = numpy.zeros(2)

    def sample():
        nonlocal state
        if kind == 'none':
            return numpy.zeros(2)
        if kind == 'bias+white':
            return numpy.array((0.0, 0.005)) + rng.normal(0, 0.02, 2)
        tau, std = NOISE[kind]
        a = math.exp(-DT / tau)
        state = a * state + math.sqrt(1 - a * a) * rng.normal(0, std, 2)
        return state
    return sample


def sim(kind, seed, obstacle=None, plant=Docked, desired=(0.0, -0.05, 0.0), current=(0, 0),
        seconds=60, drift_limit=0.3, obstacle_at=None):
    """Drift after ``seconds``; ``obstacle`` (body, m) is reported every step, or only
    while ``now < obstacle_at`` (0: once, a second before the run)."""
    rng = numpy.random.default_rng(seed)
    sample, now = noise(kind, rng), 0.0
    law = VelocityLaw(Plant(config()), drift_limit=drift_limit)
    boat = plant(current=current)
    if obstacle is not None and obstacle_at == 0.0:
        law.observe_obstacle(obstacle, -1.0)
    for _ in range(int(seconds / DT)):
        if obstacle is not None and (obstacle_at is None or now < obstacle_at):
            law.observe_obstacle(obstacle, now)
        law.observe(boat.velocity + numpy.array((*sample(), 0.0)), boat.yaw, now)
        boat.advance(law.step(desired, now), DT)
        now += DT
    return law.drift


def wound(kind, **kw):
    return sum(math.hypot(*sim(kind, seed, **kw)) > WOUND for seed in SEEDS)


@pytest.mark.parametrize('kind', ['bias+white', 'colored', 'colored10', 'colored.5'])
def test_docked_creep_does_not_wind_up_beside_the_dock(kind):
    assert wound(kind) > 0  # the carry latch alone lets this noise through
    assert wound(kind, obstacle=DOCK) == 0


@pytest.mark.parametrize('bearing', [-60, 60])
def test_a_60_degree_bearing_error_still_blocks_it(bearing):
    assert wound('colored', obstacle=rotate(DOCK, math.radians(bearing))) == 0


@pytest.mark.parametrize('current', [0.05, 0.1, 0.15, 0.3])
def test_currents_are_still_learned_in_full(current):
    # Open water (nothing near), and an obstacle the current pushes the boat toward.
    for obstacle in (None, (0.0, 0.3)):
        drift = sim('none', 0, obstacle=obstacle, plant=Boat, desired=(0.0, 0.0, 0.0),
                    current=(0.0, current), seconds=40, drift_limit=0.5)
        assert drift == pytest.approx((0.0, current), abs=0.01)


def test_a_current_toward_a_near_buoy_is_learned_through_noise():
    learned = numpy.mean([sim('colored', seed, obstacle=(0.0, 0.3), plant=Boat,
                              desired=(0.0, 0.0, 0.0), current=(0.0, 0.15), seconds=40,
                              drift_limit=0.5)[1] for seed in range(5)])
    assert learned > 0.12


@pytest.mark.parametrize('obstacle,obstacle_at', [
    (None, None),                  # never received
    (DOCK, 0.0),                   # heard only before the first odometry: stale throughout
    (DOCK, 10.0),                  # stops after 10 s: stale for the last 50
    ((math.nan, math.nan), None),  # nothing near
    ((0.0, -0.6), None),           # beyond proximity_distance
])
def test_missing_stale_or_far_obstacle_data_falls_back_to_the_latch(obstacle, obstacle_at):
    sizes = []
    for seed in SEEDS:
        drift = sim('colored', seed, obstacle=obstacle, obstacle_at=obstacle_at)
        sizes.append(math.hypot(*drift))
        if obstacle_at == 10.0:
            # Fresh for the first 10 s only: no protection after, never worse than the latch.
            assert sizes[-1] <= 0.3 + 1e-9
        elif seed < 3:
            assert drift == pytest.approx(sim('colored', seed), abs=1e-12)
    assert any(size > WOUND for size in sizes)  # no protection: the latch alone


def test_obstacle_data_that_stops_ages_out():
    law = VelocityLaw(Plant(config()))
    law.yaw = 0.0
    law.observe_obstacle(DOCK, 0.0)
    assert law.near_obstacle(law.obstacle_timeout) == pytest.approx((0.0, -1.0))
    assert law.near_obstacle(law.obstacle_timeout + 0.01) is None
    law.proximity_distance = 0.0  # off
    assert law.near_obstacle(0.0) is None
