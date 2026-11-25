"""ROS-independent planar thruster mixing and output mapping.

Conventions used throughout the package:
  * Body axes follow ROS REP 103: surge is +x forward, sway is +y port/left,
    yaw is +z counterclockwise seen from above.
  * Commands are normalized effort in [-1, 1] per axis, not m/s or rad/s.
  * Thrusters are listed in canonical order FL, FR, RR, RL. ``channel_map``
    says which physical PWM output (1-4) drives each canonical position.
"""

import math

AXES = ('surge', 'sway', 'yaw')

# ESC pulse widths in microseconds: 1500 is stopped, +/-400 is full effort.
NEUTRAL_PWM = 1500
PWM_SCALE = 400
MIN_PWM = 1100
MAX_PWM = 1900

POSITIONS = ('fl', 'fr', 'rr', 'rl')
# Rows are thrusters in POSITIONS order; columns are surge, sway, yaw.
THRUSTER_MIXER = (
    (-1.0, 1.0, 1.0),
    (-1.0, -1.0, -1.0),
    (1.0, -1.0, 1.0),
    (1.0, 1.0, -1.0),
)
# Rows and columns surge, sway, yaw: the fitted correction for cross-coupling.
IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
DECOUPLING_BOUND = 2.0  # largest |entry| of C; fitted corrections stay near identity


def axis_command(axis=None, value=0.0):
    """Return a (surge, sway, yaw) command.

    ``axis`` is an axis name driven at ``value``, an explicit three-value
    command, or None for neutral.
    """
    if isinstance(axis, (tuple, list)):
        return tuple(float(component) for component in axis)
    return tuple(float(value) if name == axis else 0.0 for name in AXES)


def effort_to_pulse(effort):
    """Convert one normalized thruster effort into a clamped ESC pulse (us)."""
    return max(MIN_PWM, min(MAX_PWM, round(NEUTRAL_PWM + PWM_SCALE * effort)))


SINGLE_MAX = 0.6        # the most effort the one-thruster test may ask of a thruster
SINGLE_TIMEOUT = 0.3    # s: a one-thruster command older than this is dropped (the keyboard repeats it)


def single_pulses(index, effort, channel_map, reversed=(False,) * 4):
    """Pulses (physical outputs 1-4) that run one thruster alone: canonical ``index`` (0 FL, 1 FR, 2 RR,
    3 RL) at ``effort`` (clamped to +/-SINGLE_MAX), the others neutral; reversed thrusters flip as in
    mix_stages, so a positive effort is that thruster's own forward thrust."""
    index = int(index)
    if not 0 <= index < 4 or not math.isfinite(effort):
        raise ValueError(f'single thruster {index} at {effort}: index 0-3 and a finite effort')
    efforts = [0.0] * 4
    efforts[index] = max(-SINGLE_MAX, min(SINGLE_MAX, float(effort)))
    flipped = [-e if flip else e for e, flip in zip(efforts, reversed)]
    return map_channels([effort_to_pulse(e) for e in flipped], channel_map)


def single_allowed(values, mux, age):
    """Whether the one-thruster test may drive: nothing else commands (the applied effort is zero), the
    mux has no lane but the keyboard's or none (never auto or the remote), holds nothing, and the
    command is fresh. ``mux`` is the mux's latched state (dict) or None (unknown: refused)."""
    if age is None or age > SINGLE_TIMEOUT or any(abs(v) > 1e-9 for v in values) or not isinstance(mux, dict):
        return False
    return mux.get('input') in ('manual', 'idle') and not mux.get('hold')


def channel_map_from_observations(observations):
    """Invert {physical channel: canonical position} into canonical order."""
    if set(observations) != {1, 2, 3, 4}:
        raise ValueError('all four physical outputs must be identified')
    if set(observations.values()) != set(POSITIONS):
        raise ValueError('each canonical position must be assigned exactly once')
    return tuple(next(channel for channel, position in observations.items()
                      if position == expected) for expected in POSITIONS)


def validate_mixer(values):
    """Return a four-row mixer from a flat, finite 12-value sequence."""
    if len(values) != 12:
        raise ValueError('thruster_matrix must contain exactly 12 values')
    if any(type(value) not in (int, float) for value in values):
        raise ValueError('matrix values must be numbers')
    values = tuple(float(value) for value in values)
    if not all(value == value and abs(value) != float('inf')
               for value in values):
        raise ValueError('thruster_matrix values must be finite')
    return tuple(tuple(values[index:index + 3])
                 for index in range(0, 12, 3))


def validate_decoupling(values):
    """Return a 3x3 decoupling matrix from a flat, finite 9-value sequence.

    Every entry lies within +/-DECOUPLING_BOUND and the diagonal is positive: a zero or
    negative diagonal would drop or reverse an axis, which no correction means to do.
    """
    if len(values) != 9:
        raise ValueError('decoupling must contain exactly 9 values')
    if any(type(value) not in (int, float) for value in values):
        raise ValueError('matrix values must be numbers')
    values = tuple(float(value) for value in values)
    if not all(value == value and abs(value) != float('inf') for value in values):
        raise ValueError('decoupling values must be finite')
    if any(abs(value) > DECOUPLING_BOUND for value in values):
        raise ValueError(f'decoupling values must be within +/-{DECOUPLING_BOUND:g}')
    if any(values[index] <= 0 for index in (0, 4, 8)):
        raise ValueError('decoupling diagonal must be positive')
    return tuple(tuple(values[index:index + 3]) for index in range(0, 9, 3))


def decouple(command, decoupling=IDENTITY, negative=None):
    """Apply the decoupling: e' = C e, so asking for one axis yields only that axis.

    With ``negative``, an axis commanded below zero takes its column from that
    matrix instead. Coupling that acts in one direction only (drag at the leading
    corner of the hull, say) needs a different column for each sign; a single
    linear C would fix one direction and worsen the other.
    """
    negative = decoupling if negative is None else negative
    return tuple(sum((row if command[j] >= 0 else below)[j] * command[j] for j in range(3))
                 for row, below in zip(decoupling, negative))


def validate_channel_map(values):
    """Validate canonical-to-physical output numbers."""
    if any(type(value) is not int for value in values):
        raise ValueError('channel_map must contain integers')
    channels = tuple(values)
    if len(channels) != 4 or sorted(channels) != [1, 2, 3, 4]:
        raise ValueError('channel_map must be a permutation of [1, 2, 3, 4]')
    return channels


def validate_reversed(values):
    """Four booleans in FL, FR, RR, RL order: True flips that thruster's effort."""
    if len(values) != 4 or any(type(value) is not bool for value in values):
        raise ValueError('reversed must contain four booleans (FL, FR, RR, RL)')
    return tuple(values)


def validate_reverse_ratio(value):
    """Full reverse over full forward thrust, in [0.5, 1] (schema.yaml's range): one ratio for all four thrusters."""
    if type(value) not in (int, float) or not 0.5 <= value <= 1.0:
        raise ValueError('reverse_thrust_ratio must be a number in [0.5, 1]')
    return float(value)


def thruster_pulses(command, mixer, channel_map, reversed=(False,) * 4, decoupling=IDENTITY,
                    decoupling_negative=None, reverse_ratio=1.0):
    """Effort command -> ESC pulse per physical output (outputs 1-4 in order); see mix_stages."""
    return mix_stages(command, mixer, channel_map, reversed, decoupling, decoupling_negative,
                      reverse_ratio)['outputs']


def mix_stages(command, mixer=THRUSTER_MIXER, channel_map=(1, 2, 3, 4), reversed=(False,) * 4,
               decoupling=IDENTITY, decoupling_negative=None, reverse_ratio=1.0):
    """Every stage from an effort command to the physical pulses (docs/CONTROL.md L0-L3).

    Decouple the axes, mix and desaturate, flip reversed thrusters (a motor
    wired or a propeller mounted backwards), convert to pulses, then route each
    position to its output. Efforts are normalised, so the command is clamped
    to [-1, 1] first: a huge finite value must not overflow the mixer into NaN.
    ``reverse_ratio`` is the thrust below 1500 us relative to above it. The driver
    uses only ``outputs``; ``mix`` prints them all.
    """
    command = tuple(max(-1.0, min(1.0, component)) for component in command)
    # The weak side of a flipped thruster is its positive canonical effort.
    ratios = [1.0 / reverse_ratio if flip else reverse_ratio for flip in reversed]
    stages = _mix(command, mixer, ratios, decoupling, decoupling_negative)
    flipped = [-effort if flip else effort for effort, flip in zip(stages['effort'], reversed)]
    pulses = [effort_to_pulse(effort) for effort in flipped]
    return dict(stages, flipped=flipped, pulses=pulses, outputs=map_channels(pulses, channel_map))


def map_channels(values, channel_map):
    """Map canonical FL, FR, RR, RL values onto physical PWM outputs."""
    mapped = [None] * 4
    for value, physical_channel in zip(values, channel_map):
        mapped[physical_channel - 1] = value
    return mapped


def mix_thrusters(surge, sway, yaw, mixer=THRUSTER_MIXER, reverse_ratio=(1.0,) * 4,
                  decoupling=IDENTITY, decoupling_negative=None):
    """Normalized, desaturated efforts in FL, FR, RR, RL order (before reversal); see _mix."""
    return _mix((surge, sway, yaw), mixer, reverse_ratio, decoupling, decoupling_negative)['effort']


def thrust_of(effort, ratio=1.0):
    """L1 thruster law: thrust goes as e|e|, ``ratio`` weaker below zero."""
    return effort * abs(effort) * (ratio if effort < 0 else 1.0)


def effort_of(force, ratio=1.0):
    """The inverse of thrust_of."""
    return math.copysign(math.sqrt(abs(force) / (ratio if force < 0 else 1.0)), force)


def _mix(command, mixer, ratios, decoupling, decoupling_negative):
    """The L3 and L2 stages, per thruster ratio.

    Mixed in thrust, not effort: each thruster sums the thrust every axis alone
    (decoupled as before) would ask of it and takes the effort that produces the
    total. A single axis is unchanged; a combined command is the sum of its
    single-axis wrenches, so a diagonal neither turns (unequal forward and reverse
    thrust on a diagonal pair) nor bends toward the larger axis. Summing efforts
    instead gets both wrong.
    """
    columns = ['C-' if value < 0 else 'C' for value in command]
    parts = [decouple(axis_command(axis, value), decoupling, decoupling_negative)
             for axis, value in zip(AXES, command)]
    # per_axis[i][k]: the effort thruster i would take for axis k alone (row i of M times part k).
    per_axis = [[sum(gain * part[j] for j, gain in enumerate(row)) for part in parts]
                for row in mixer]
    thrust = [sum(thrust_of(value, ratio) for value in row)
              for row, ratio in zip(per_axis, ratios)]
    raw = [effort_of(force, ratio) for force, ratio in zip(thrust, ratios)]
    # Scale every thruster down together when any would exceed full effort: every
    # thrust shrinks by the same factor, so the wrench keeps its direction.
    peak = max(1.0, *(abs(value) for value in raw))
    return dict(command=command, columns=columns, parts=parts, per_axis=per_axis,
                thrust=thrust, raw=raw, peak=peak, effort=[value / peak for value in raw])
