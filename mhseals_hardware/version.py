"""Which mhseals_hardware code a process runs: a digest of its sources, and the git commit when it has one.

The driver reports it in ~/output and the TUI compares it with its own copy, so a driver still
running code from before a deploy (it loads new code only when the next arm starts it) shows.
The digest covers the package's .py files as this process would import them; it is the same on
any host that has the same code, with or without a git checkout (the sim overlay has none).
"""

import functools
import hashlib
from pathlib import Path
import subprocess

PACKAGE = Path(__file__).resolve().parent


@functools.lru_cache(maxsize=None)
def code_version(package=PACKAGE):
    """{'digest': 12 hex of the sources, 'git': short commit or None, 'dirty': bool or None}."""
    package = Path(package).resolve()
    digest = hashlib.sha256()
    for path in sorted(package.glob('*.py')):
        digest.update(path.name.encode())
        digest.update(path.read_bytes())
    git = dirty = None
    try:
        run = functools.partial(subprocess.run, cwd=package, capture_output=True, text=True,
                                timeout=2.0, check=True)
        git = run(['git', 'rev-parse', '--short', 'HEAD']).stdout.strip() or None
        dirty = bool(run(['git', 'status', '--porcelain', '--', '.']).stdout.strip())
    except (OSError, subprocess.SubprocessError):
        pass
    return {'digest': digest.hexdigest()[:12], 'git': git, 'dirty': dirty}
