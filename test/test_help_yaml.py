"""help.yaml lists every console script, and only the options they parse."""
from pathlib import Path
import re

import yaml

ROOT = Path(__file__).resolve().parents[1]
SHARED = (ROOT / 'mhseals_hardware/configuration.py').read_text()


def scripts():
    """{console script: its module source} from setup.py."""
    found = re.findall(r"'(\w+) = mhseals_hardware\.(\w+):main'",
                       (ROOT / 'setup.py').read_text())
    return {name: (ROOT / 'mhseals_hardware' / (module + '.py')).read_text()
            for name, module in found}


def test_every_console_script_is_listed_with_real_options():
    commands = yaml.safe_load((ROOT / 'help.yaml').read_text())['commands']
    listed = {c['run'].split()[3]: c for c in commands
              if c['run'].startswith('ros2 run mhseals_hardware ')}
    assert set(listed) == set(scripts())
    for name, source in scripts().items():
        if 'configure_args' in source:
            source += SHARED
        for param in listed[name].get('params', []):
            assert "'%s'" % param['name'] in source, (name, param['name'])
