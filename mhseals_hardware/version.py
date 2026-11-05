"""Which mhseals_hardware code a process runs: a digest of its sources.

The driver reports it in ~/output and the TUI compares it with its own copy, so a driver still
running code from before a deploy (it loads new code only when the next arm starts it) shows.
The digest covers the package's .py files as this process would import them; it is the same on
any host that has the same code, with or without a git checkout (the sim overlay has none).
Git provenance belongs to the session record, not the driver's startup path.
"""

import functools
import hashlib
from pathlib import Path

PACKAGE = Path(__file__).resolve().parent


@functools.lru_cache(maxsize=None)
def code_version(package=PACKAGE):
    """{'digest': 12 hex of the sources}."""
    digest = hashlib.sha256()
    for path in sorted(Path(package).resolve().glob('*.py')):
        digest.update(path.name.encode())
        digest.update(path.read_bytes())
    return {'digest': digest.hexdigest()[:12]}
