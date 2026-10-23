"""Guard: the secant-gain and single-C helpers were retired with the speed curves."""
import pathlib
import re

REMOVED = ('raw_gain', 'decoupling_for', 'fitted_with_decoupling', 'single_decoupling')
ROOT = pathlib.Path(__file__).resolve().parent.parent


def test_removed_helper_names_do_not_come_back():
    pattern = re.compile('|'.join(REMOVED))
    here = pathlib.Path(__file__).resolve()
    found = [f'{path.relative_to(ROOT)}:{number}'
             for path in [*ROOT.glob('mhseals_hardware/**/*'), *ROOT.glob('test/**/*'),
                          *ROOT.glob('*.md')]
             if path.is_file() and path != here and path.suffix in ('.py', '.yaml', '.md')
             for number, line in enumerate(path.read_text().splitlines(), 1)
             if pattern.search(line)]
    assert not found, found
