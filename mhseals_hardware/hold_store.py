"""Persist the command_mux hold set so a restarted mux keeps holding.

The file is tiny JSON, replaced atomically.  A missing, torn or malformed
file reads as "no holds": the mux then starts clean and ramps auto in.
"""

import json
import os
import tempfile
from pathlib import Path


def default_path():
    """``$XDG_RUNTIME_DIR/mhseals/command_mux_<domain>.json`` (temp dir if unset)."""
    base = os.environ.get('XDG_RUNTIME_DIR') or tempfile.gettempdir()
    domain = os.environ.get('ROS_DOMAIN_ID') or '0'
    return Path(base) / 'mhseals' / ('command_mux_%s.json' % domain)


def save(path, holds):
    """Atomically write ``holds`` (cause -> {cause, by, since, token hash})."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + '.tmp')
    temp.write_text(json.dumps(sorted(holds.values(), key=lambda h: h['since'])))
    os.replace(temp, path)


def load(path):
    """Return the saved holds as a dict, or ``{}`` when unreadable."""
    try:
        rows = json.loads(Path(path).read_text())
        return {row['cause']: {'cause': str(row['cause']), 'by': str(row['by']),
                               'since': float(row['since']),
                               'token': row['token'] if isinstance(row['token'], str) else None}
                for row in rows}
    except (OSError, ValueError, KeyError, TypeError):
        return {}
