"""Thruster output backends selected by the ``output_backend`` config key.

``odroid`` drives the ODROID's PWM pins. ``topic`` publishes the same four
pulse widths on a ROS topic for a simulator (crane_ml), so the simulated boat
receives exactly what the pins would: after the mixer, the channel map and the
calibration. Both share one interface: ``open``, ``set_pulse_widths``,
``neutral``, ``close``.
"""

import threading

from mhseals_hardware.odroid_pwm import OdroidPWMOutputs
from mhseals_hardware.thruster_mixer import MAX_PWM, MIN_PWM, NEUTRAL_PWM

BACKENDS = ('odroid', 'topic')
# A simulator treats a silent topic as a lost link, so the last pulses are
# repeated even when the commands stop changing.
REPUBLISH_HZ = 20.0


class TopicPWMOutputs:
    """Publish physical-output pulse widths (outputs 1-4, µs) as UInt16MultiArray.

    With ``node`` the publisher lives on that node (thruster_pwm_node);
    without, ``open`` starts a private node and spin thread, for standalone
    tools such as thruster_identify.
    """

    def __init__(self, topic='/thrusters/pwm', node=None, neutral_us=NEUTRAL_PWM,
                 message_type=None):
        self.topic = topic
        self.node = node
        self.neutral_us = int(neutral_us)
        self.message_type = message_type
        self.pulses = [self.neutral_us] * 4
        self.publisher = self.timer = self.executor = self.thread = None
        self.own_node = node is None
        self.started_rclpy = False

    def open(self):
        if self.message_type is None:
            from std_msgs.msg import UInt16MultiArray
            self.message_type = UInt16MultiArray
        if self.own_node:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            if not rclpy.ok():
                rclpy.init()
                self.started_rclpy = True
            self.node = rclpy.create_node('thruster_pwm_outputs')
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.node)
            self.thread = threading.Thread(target=self.executor.spin, daemon=True)
            self.thread.start()
        self.publisher = self.node.create_publisher(self.message_type, self.topic, 10)
        self.timer = self.node.create_timer(1.0 / REPUBLISH_HZ, self._publish)
        self.neutral()
        return self

    def _publish(self):
        self.publisher.publish(self.message_type(data=list(self.pulses)))

    def set_pulse_widths(self, values):
        values = [int(round(value)) for value in values]
        if len(values) != 4:
            raise ValueError('exactly four pulse widths are required')
        if any(not MIN_PWM <= value <= MAX_PWM for value in values):
            raise ValueError(f'pulse widths must be within {MIN_PWM}-{MAX_PWM} µs')
        self.pulses = values
        self._publish()

    def neutral(self):
        self.set_pulse_widths([self.neutral_us] * 4)

    def close(self, unexport=False):
        if self.publisher is None:
            return
        self.node.destroy_timer(self.timer)
        for _ in range(3):  # neutral last, repeated in case one is dropped
            self.neutral()
        self.node.destroy_publisher(self.publisher)
        self.publisher = None
        if self.own_node:
            self.executor.shutdown(timeout_sec=1)
            self.node.destroy_node()
            if self.started_rclpy:
                import rclpy
                rclpy.try_shutdown()


OUTPUT_KEYS = ('output_backend', 'pwm_topic', 'pwm_chips', 'pwm_channels', 'mosfet_chip',
               'mosfet_line', 'mosfet_active_high')


def switch_outputs(outputs, old, new, node=None, make=None):
    """Move live outputs to new pins or backend; returns (outputs, error).

    Neutral, ESC power off, release the old channels, open the new ones at
    neutral, power on. If the new outputs fail to open, the old ones reopen.
    Always attempt release after neutral; any cleanup failure prevents opening.
    """
    make = make or make_outputs
    neutral_error = None
    try:
        outputs.neutral()
    except Exception as error:
        neutral_error = error
    try:
        outputs.close(unexport=True)
    except Exception as error:
        if neutral_error is not None:
            raise OSError(
                f'outputs neutral failed ({type(neutral_error).__name__}): {neutral_error}; '
                f'close failed ({type(error).__name__}): {error}') from error
        raise
    if neutral_error is not None:
        raise neutral_error
    try:
        return make(new, node=node).open(), None
    except Exception as error:
        return make(old, node=node).open(), error


def make_outputs(config, node=None):
    """Thruster outputs for a hardware config; ``node`` hosts a topic publisher."""
    backend = config['output_backend']
    if backend == 'topic':
        return TopicPWMOutputs(config['pwm_topic'], node)
    return OdroidPWMOutputs.from_config(config)
