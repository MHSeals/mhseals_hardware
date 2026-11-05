"""Every docs/authority.md citation in the package names a section that exists."""

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCES = [p for d in ('mhseals_hardware', 'config', 'host') for p in (ROOT / d).rglob('*')
           if p.is_file() and p.suffix in ('.py', '.yaml', '')] + [ROOT / 'README.md']


def cited():
    for path in SOURCES:
        text = path.read_text(errors='ignore')
        assert '.handoff/specs' not in text, path
        for match in re.finditer(r'docs/authority\.md (invariant )?(\d+(?:\.\d+)?)', text):
            yield path, bool(match.group(1)), match.group(2)


def test_every_cited_section_exists():
    doc = (ROOT / 'docs' / 'authority.md').read_text()
    sections = set(re.findall(r'^#+ (\d+(?:\.\d+)?)\.? ', doc, re.M))
    invariants = set(re.findall(r'^(\d+)\. \*\*', doc.split('## 2.')[0], re.M))
    found = list(cited())
    assert found
    for path, invariant, number in found:
        assert number in (invariants if invariant else sections), (path, number)


def test_the_mux_order_always_leads_with_duplicate():
    """A duplicate hold outranks rc (docs/authority.md 2.2), so no summary of the order drops it."""
    for path in SOURCES + [ROOT / 'help.yaml']:
        text = path.read_text(errors='ignore')
        for match in re.finditer(r'rc > manual', text):
            assert text[:match.start()].endswith('duplicate > '), path
