"""Arrow-key option picker shared by the interactive hardware tools."""

from rich.live import Live
from rich.panel import Panel

from mhseals_hardware.keyboard import KeyReader


class Cancelled(RuntimeError):
    """The operator cancelled a prompt (Q or Escape)."""


def select_option(console, title, options, shortcuts=None, open_keys=KeyReader, live=Live):
    """Return one of ``options``, chosen with arrows and Enter or a shortcut key.

    ``shortcuts`` maps a key to an option index. Q or Escape raises
    ``Cancelled``. ``open_keys`` and ``live`` are injectable for tests.
    """
    selected = 0
    shortcuts = shortcuts or {}

    def render():
        rows = [f"{'[bold cyan]›[/]' if index == selected else ' '} {option}"
                for index, option in enumerate(options)]
        rows.append('\n[dim]↑/↓ select  •  Enter confirm[/]')
        return Panel('\n'.join(rows), title=title, expand=False)

    with open_keys() as keys, live(render(), console=console, refresh_per_second=10) as view:
        while True:
            key = keys.read(timeout=0.1)
            if key == 'up':
                selected = (selected - 1) % len(options)
            elif key == 'down':
                selected = (selected + 1) % len(options)
            elif key == 'enter':
                return options[selected]
            elif key in shortcuts:
                return options[shortcuts[key]]
            elif key in ('q', 'escape'):
                raise Cancelled('selection cancelled')
            view.update(render())
