import pytest

from mhseals_hardware.configuration import load_defaults, validate_config
from mhseals_hardware.odroid_pwm import OdroidPWMOutputs
from mhseals_hardware.outputs import TopicPWMOutputs, make_outputs


class Message:
    def __init__(self, data):
        self.data = data


class Node:
    def __init__(self):
        self.published, self.timers, self.destroyed = [], [], []

    def create_publisher(self, message_type, topic, depth):
        self.topic = topic
        return self

    def publish(self, message):
        self.published.append(message.data)

    def create_timer(self, period, callback):
        self.timers.append((period, callback))
        return callback

    def destroy_timer(self, timer):
        self.destroyed.append('timer')

    def destroy_publisher(self, publisher):
        self.destroyed.append('publisher')


def test_topic_outputs_publish_neutral_first_and_last():
    node = Node()
    outputs = TopicPWMOutputs('/sim/pwm', node, message_type=Message).open()
    assert node.topic == '/sim/pwm' and node.published == [[1500] * 4]
    outputs.set_pulse_widths([1600, 1400, 1500.4, 1900])
    assert node.published[-1] == [1600, 1400, 1500, 1900]
    period, republish = node.timers[0]
    republish()  # the watchdog-feeding repeat sends the last pulses again
    assert period == pytest.approx(0.05) and node.published[-1] == [1600, 1400, 1500, 1900]
    outputs.close()
    assert node.published[-3:] == [[1500] * 4] * 3
    assert node.destroyed == ['timer', 'publisher']
    outputs.close()  # idempotent


@pytest.mark.parametrize('values', [[1500] * 3, [1500, 1500, 1500, 2100], [1000] * 4])
def test_topic_outputs_reject_bad_pulses(values):
    outputs = TopicPWMOutputs(node=Node(), message_type=Message).open()
    with pytest.raises(ValueError):
        outputs.set_pulse_widths(values)


def test_make_outputs_selects_backend():
    config = load_defaults()
    assert isinstance(make_outputs(config), OdroidPWMOutputs)
    outputs = make_outputs(dict(config, output_backend='topic'), node=Node())
    assert isinstance(outputs, TopicPWMOutputs) and outputs.topic == '/thrusters/pwm'


@pytest.mark.parametrize('key,value', [('output_backend', 'gpio'), ('pwm_topic', 'relative')])
def test_config_validates_backend(key, value):
    with pytest.raises(ValueError, match=key):
        validate_config(dict(load_defaults(), **{key: value}))
