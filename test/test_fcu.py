"""Tests for deterministic MAVROS serial-device discovery."""

from mhseals_hardware.fcu import default_fcu_url, detect_fcu_device


def test_prefers_hinted_stable_device(tmp_path):
    by_id = tmp_path / 'serial' / 'by-id'
    by_id.mkdir(parents=True)
    (by_id / 'usb-camera').touch()
    pixhawk = by_id / 'usb-Holybro_Pixhawk'
    pixhawk.touch()
    assert detect_fcu_device(tmp_path) == pixhawk


def test_ignores_unknown_stable_device(tmp_path):
    by_id = tmp_path / 'serial' / 'by-id'
    by_id.mkdir(parents=True)
    device = tmp_path / 'ttyACM0'
    device.touch()
    controller = by_id / 'usb-flight-controller'
    controller.symlink_to(device)
    assert detect_fcu_device(tmp_path) is None


def test_falls_back_to_first_tty_device(tmp_path):
    device = tmp_path / 'ttyACM1'
    device.touch()
    assert detect_fcu_device(tmp_path) == device


def test_url_has_mavros_serial_format(tmp_path):
    (tmp_path / 'ttyACM0').touch()
    assert default_fcu_url(tmp_path) == f'serial://{tmp_path}/ttyACM0:57600'
