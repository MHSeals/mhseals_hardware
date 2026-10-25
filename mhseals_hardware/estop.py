"""Remote e-stop: the RC stop-channel edge cuts ESC power through the MOSFET.

ROS-free and clock-injected. The edge is the command_mux rule itself
(``Arbitration.update_rc``), fed the same ``command_mux`` section. The cut
latches for the life of the driver: nothing here powers the ESCs again. Power
returns only through the arm path, which starts a new driver whose
``outputs.open()`` re-enables the MOSFET with every channel at neutral.
"""

from mhseals_hardware.command_arbitration import Arbitration


class RemoteEstop:
    """Watch RC frames; on the stop edge cut the MOSFET, go neutral, and latch."""

    def __init__(self, outputs, mux_section, enabled=True):
        self.outputs = outputs
        self.enabled = bool(enabled)
        self.section = None
        self.configure(mux_section)
        self.cause = None
        self.since_ns = None

    @property
    def latched(self):
        return self.cause is not None

    def configure(self, mux_section):
        """Follow command_mux edits; an unchanged section keeps the previous frame."""
        if mux_section != self.section:
            self.rule = Arbitration.from_config(mux_section)
            self.section = mux_section

    def rc(self, channels, now, wall_ns):
        """Feed one RC frame; True when this frame cut the power."""
        edge = self.rule.update_rc(list(channels), now)
        if edge and self.enabled and not self.latched:
            self.cut('rc-stop', wall_ns)
            return True
        return False

    def cut(self, cause, wall_ns):
        """Power off first, then neutral; latch even if a write fails, then re-raise."""
        self.cause, self.since_ns = cause, wall_ns
        mosfet = getattr(self.outputs, 'mosfet', None)
        error = None
        if mosfet is not None and mosfet.request is not None:
            try:
                mosfet.set_enabled(False)
            except Exception as failure:
                error = failure
        try:
            self.outputs.neutral()
        except Exception as failure:
            error = error or failure
        if error is not None:
            raise error

    def state(self):
        """For the driver ~/status: whether the cut is armed, and the latch."""
        return {'enabled': self.enabled, 'latched': self.latched,
                'cause': self.cause or '', 'since_ns': self.since_ns}
