"""Locate data belonging to this code checkout/install without ROS imports."""
from pathlib import Path


def config_resource(name):
    """Select this source tree or its own install prefix, never another overlay.

    Symlink installs resolve to their source tree. Wheels (including --target)
    and copied colcon installs use share/mhseals_hardware in the code's prefix.
    Missing selected data is an error, not permission to search other installs.
    """
    if name not in ('default.yaml', 'schema.yaml'):
        raise ValueError('unknown hardware configuration resource: ' + name)
    package = Path(__file__).resolve().parent
    root = package.parent
    if (root / 'package.xml').is_file() and (root / 'resource/mhseals_hardware').is_file():
        directory = root / 'config'
    else:
        # pip --target places the module and share directly under the target.
        prefix = root
        if root.name in ('site-packages', 'dist-packages'):
            library = next((p for p in root.parents if p.name.lower() in ('lib', 'lib64')), None)
            if library is None:
                raise FileNotFoundError('cannot determine hardware install prefix: ' + str(root))
            prefix = library.parent
        directory = prefix / 'share/mhseals_hardware/config'
    resource = directory / name
    if not resource.is_file():
        raise FileNotFoundError('required hardware configuration resource missing: ' + str(resource))
    return resource
