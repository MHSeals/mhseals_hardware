"""Pure controller domains shared by startup and parameter preflight."""
from array import array
import math

VECTOR_GAINS = ('kp', 'ki', 'integral_limit')
SCALAR_GAINS = ('drift_time_constant', 'odom_timeout', 'drift_command_limit', 'drift_limit',
                'drift_carry', 'proximity_distance', 'obstacle_timeout')
GAIN_KEYS = VECTOR_GAINS + SCALAR_GAINS


def nonnegative(value, name, positive=False):
    if type(value) not in (int, float) or not math.isfinite(value) or (
            value <= 0 if positive else value < 0):
        raise ValueError(f'{name} must be finite and {"positive" if positive else "nonnegative"}')


def validate_controller(values):
    for name in VECTOR_GAINS:
        value = values[name]
        if not isinstance(value, (list, tuple, array)) or len(value) != 3:
            raise ValueError(f'{name} needs three numbers')
        for item in value:
            nonnegative(item, name)
    for name in SCALAR_GAINS:
        nonnegative(values[name], name, positive=name == 'odom_timeout')
    if type(values['feedback']) is not bool:
        raise ValueError('feedback must be a boolean')
    return values


def validate_drift_limit(drift_limit, limits):
    """Startup: drift_limit must sit below the slower of surge and sway, else contact with a
    dock can still wind the effort up to full before the backstop bites."""
    slowest = min(limits[:2])
    if drift_limit >= slowest:
        raise ValueError(f'drift_limit {drift_limit} m/s must be below the slowest planar '
                         f'velocity limit {slowest} m/s')


# The controller's own sensor inputs; its command lanes come from the topics: section.
TOPIC_KEYS = ('odom_topic', 'obstacle_topic')
CONTROLLER_KEYS = GAIN_KEYS + ('feedback',) + TOPIC_KEYS
MOTION_TOPICS = ('auto', 'effort', 'manual', 'rc', 'applied', 'mux_state')


def ros_topic(topic, name):
    """Refuse anything but an absolute ROS topic name."""
    import re

    if not isinstance(topic, str) or not re.fullmatch(
            r'/(?:[A-Za-z_][A-Za-z_0-9]*)(?:/[A-Za-z_][A-Za-z_0-9]*)*', topic):
        raise ValueError(f'{name} must be an absolute ROS topic')


def topics_section(values):
    """The topics: section names every motion topic, each absolute and distinct."""
    if not isinstance(values, dict) or set(values) != set(MOTION_TOPICS):
        raise ValueError('topics must name exactly ' + ', '.join(MOTION_TOPICS))
    seen = {}
    for name in MOTION_TOPICS:
        ros_topic(values[name], f'topics.{name}')
        if values[name] in seen:    # one topic on two lanes loops a node's output into its own input
            raise ValueError(f'topics.{seen[values[name]]} and topics.{name} both name {values[name]}')
        seen[values[name]] = name
    return values


def controller_section(values):
    """Validate a complete owned section and normalize ROS numeric types."""
    if not isinstance(values, dict) or set(values) != set(CONTROLLER_KEYS):
        raise ValueError('controller must contain exactly the registered controller keys')
    validate_controller(values)
    for name in TOPIC_KEYS:
        ros_topic(values[name], f'controller.{name}')
    return {name: [float(v) for v in value] if name in VECTOR_GAINS
            else float(value) if name in SCALAR_GAINS else value
            for name, value in values.items()}
