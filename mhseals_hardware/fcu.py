"""MAVROS flight-controller serial discovery."""

from pathlib import Path


DEFAULT_FCU_DEVICE = Path('/dev/ttyACM0')
DEFAULT_FCU_BAUD = 57600
FCU_ID_HINTS = ('ardupilot', 'pixhawk', 'px4', 'cube', 'holybro')


def detect_fcu_device(dev_root=Path('/dev')):
    """Return the most likely stable FCU device path, if one exists."""
    dev_root = Path(dev_root)
    by_id = dev_root / 'serial' / 'by-id'
    if by_id.is_dir():
        links = sorted(path for path in by_id.iterdir()
                       if path.is_symlink() or path.is_file())
        hinted = [path for path in links
                  if any(hint in path.name.lower() for hint in FCU_ID_HINTS)]
        if hinted:
            return hinted[0]
        if len(links) == 1:
            return links[0]

    candidates = sorted((*dev_root.glob('ttyACM*'), *dev_root.glob('ttyUSB*')))
    return candidates[0] if candidates else None


def default_fcu_url(dev_root=Path('/dev')):
    """Build a MAVROS serial URL from discovery or the standard fallback."""
    device = detect_fcu_device(dev_root) or DEFAULT_FCU_DEVICE
    return f'serial://{device}:{DEFAULT_FCU_BAUD}'
