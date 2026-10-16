"""ROS-independent planar thruster mixing and output mapping.

Conventions used throughout the package:
  * Body axes follow ROS REP 103: surge is +x forward, sway is +y port/left,
    yaw is +z counterclockwise seen from above.
  * Commands are normalized effort in [-1, 1] per axis, not m/s or rad/s.
  * Thrusters are listed in canonical order FL, FR, RR, RL. ``channel_map``
    says which physical PWM output (1-4) drives each canonical position.
"""

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
    values = tuple(float(value) for value in values)
    if not all(value == value and abs(value) != float('inf')
               for value in values):
        raise ValueError('thruster_matrix values must be finite')
    return tuple(tuple(values[index:index + 3])
                 for index in range(0, 12, 3))


def validate_channel_map(values):
    """Validate canonical-to-physical output numbers."""
    channels = tuple(int(value) for value in values)
    if len(channels) != 4 or sorted(channels) != [1, 2, 3, 4]:
        raise ValueError('channel_map must be a permutation of [1, 2, 3, 4]')
    return channels


def validate_reversed(values):
    """Four booleans in FL, FR, RR, RL order: True flips that thruster's effort."""
    if len(values) != 4 or any(type(value) is not bool for value in values):
        raise ValueError('reversed must contain four booleans (FL, FR, RR, RL)')
    return tuple(values)


def thruster_pulses(command, mixer, channel_map, reversed=(False,) * 4):
    """Effort command -> ESC pulse per physical output (outputs 1-4 in order).

    Mix and desaturate, flip reversed thrusters (a motor wired or a propeller
    mounted backwards), convert to pulses, then route each position to its output.
    """
    efforts = [-effort if flip else effort
               for effort, flip in zip(mix_thrusters(*command, mixer), reversed)]
    return map_channels([effort_to_pulse(effort) for effort in efforts], channel_map)


def map_channels(values, channel_map):
    """Map canonical FL, FR, RR, RL values onto physical PWM outputs."""
    mapped = [None] * 4
    for value, physical_channel in zip(values, channel_map):
        mapped[physical_channel - 1] = value
    return mapped


def mix_thrusters(surge, sway, yaw, mixer=THRUSTER_MIXER):
    """Return normalized, desaturated outputs in FL, FR, RR, RL order."""
    commands = [row[0] * surge + row[1] * sway + row[2] * yaw
                for row in mixer]
    # Scale every thruster down together when any would exceed full effort,
    # so the commanded direction is kept and only its magnitude shrinks.
    peak = max(1.0, *(abs(command) for command in commands))
    return [command / peak for command in commands]
