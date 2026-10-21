"""The pilot's console: the radio, the switch panel and keys driving the launcher.

`swarm pilot` and `swarm fly` tick it with the USB radio (`swarm.radio`) and the switch panel
(`swarm.panel`); `swarm rehearse` ticks the same code with scripted stand-ins for both.
"""

import logging
import time
from collections.abc import Callable
from typing import Protocol

from swarm.launch import Launcher
from swarm.panel import PanelControl, PanelState

log = logging.getLogger(__name__)


class PanelDevice(Protocol):
    def poll(self) -> PanelState | None:
        """The switch positions if they changed since the last call, else None."""
        ...

    def set_leds(self, value: int) -> None: ...


class RadioDevice(Protocol):
    def mode(self) -> int:
        """The mode the flight-mode switch asks for."""
        ...

    def retarget(self, sysid: int | None) -> None:
        """Fly copter `sysid`, or none."""
        ...


class Console:
    def __init__(
        self,
        launcher: Launcher,
        panel: PanelDevice | None = None,
        radio: RadioDevice | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.launcher = launcher
        self.panel = panel
        self.radio = radio
        """Set once the radio is found; it may be attached late."""
        self.clock = clock
        self.control: PanelControl | None = None
        """The switch panel's rules, from its first report on."""

    def tick(self, keys: str = "") -> None:
        launcher = self.launcher
        launcher.link.poll()
        state = self.panel.poll() if self.panel is not None else None
        if state is not None:
            if self.control is None:
                log.info("switch panel in charge: %s", state.describe())
                self.control = PanelControl(launcher, self.clock)
            self.control.update(state)
        if self.control is not None:
            self.control.step()
        for key in keys:
            launcher.press(key)
        launcher.step()
        if self.radio is not None:
            launcher.mode_switch(self.radio.mode())
            self.radio.retarget(launcher.radio_sysid)
        if self.panel is not None and self.control is not None:
            self.panel.set_leds(self.control.leds())
