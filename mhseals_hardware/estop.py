"""E-stop: the RC stop-channel edge, the operator's ``~/estop`` or a stalled heartbeat cuts
ESC power through the MOSFET. Losing RC never cuts: the mux falls through and the command
timeout brings neutral.

ROS-free and clock-injected. The edge is the command_mux rule itself
(``Arbitration.update_rc``), built once from the config's ``mux_section``, the
same one the mux reads, so the two never disagree on the stop or mode channel. The cut
latches for the life of the driver: nothing here powers the ESCs again. Power
returns only when the driver restarts: a new driver's
``outputs.open()`` re-enables the MOSFET with every channel at neutral.
"""

from mhseals_hardware.command_arbitration import Arbitration, mux_section


class RemoteEstop:
    """Watch RC frames; on the stop edge cut the MOSFET, go neutral, and latch."""

    def __init__(self, outputs, config, enabled=True):
        self.outputs = outputs
        self.enabled = bool(enabled)
        self.rule = Arbitration.from_config(mux_section(config))
        self.cause = None
        self.since_ns = None

    @property
    def latched(self):
        return self.cause is not None

    def rc(self, channels, now, wall_ns):
        """Feed one RC frame; True when this frame cut the power."""
        edge = self.rule.update_rc(list(channels), now)
        if edge and self.enabled and not self.latched:
            self.cut('rc-stop', wall_ns)
            return True
        return False

    def trip(self, cause, now, wall_ns):
        """A cut not from RC (the operator, a stalled heartbeat); always armed.
        An earlier cut keeps its cause. True when this call cut the power."""
        if self.latched:
            return False
        self.cut(cause, wall_ns)
        return True

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
