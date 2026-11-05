"""The radio read by the server, and flown from it (DESIGN.md §3.1, §12).

A radio read through Linux (rc.evdev) flies straight to the firmware: each stick movement is
mapped (rc.mapping: the player's calibration, else the radio's profile) and sent at once, and the
sticks are sent again every REPEAT_S between movements, as ELRS sends them every 4 ms. They never
pass through the page, whose one thread is busy drawing (in Chrome they took 97 ms from a stick
step to the elevons through it).

The page still gets the axes and the channels at SHOW_HZ (protocol.RadioMsg), for the HUD, the
Controls tab, the calibration wizard and the truck, and says whether this radio flies
(protocol.RadioSourceMsg): it does unless the player chose the keyboard or another controller.
While no page has it fly, it sends nothing, and the session's failsafe takes the silence for a
lost link, as it does when the radio is unplugged.
"""

import threading
import time
from collections.abc import Callable, Mapping
from typing import Any, Protocol

from drone_duels.content.store import Content
from drone_duels.rc import mapping
from drone_duels.rc.evdev import RadioReader

REPEAT_S = 1 / 250
"""The sticks again, between movements."""
SHOW_HZ = 60.0
"""The radio as the pages see it."""


class Reader(Protocol):
    @property
    def name(self) -> str | None: ...

    def axes(self) -> list[float] | None: ...

    def start(self) -> None: ...

    def stop(self) -> None: ...


class Sticks(Protocol):
    """Where the sticks go (the session)."""

    vehicle: str | None

    def send_rc(self, us: list[int]) -> None: ...


class Pilot:
    def __init__(
        self,
        session: Sticks,
        publish: Callable[[dict[str, Any]], None],
        content: Content,
        prefs: Mapping[str, Any],
        reader: Callable[[Callable[[], None]], Reader] = RadioReader,
    ) -> None:
        self.session = session
        self.publish = publish
        self.content = content
        self._make_reader = reader
        self._calibrations = mapping.calibrations(prefs)
        self._wanted: dict[object, bool] = {}
        self._lock = threading.Lock()
        self._changed = threading.Event()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._reader: Reader | None = None

    def want(self, who: object, on: bool, fly: bool = False) -> None:
        """Page `who` wants the radio streamed (`on`), and flown (`fly`), or neither."""
        with self._lock:
            if on:
                self._wanted[who] = fly
            else:
                self._wanted.pop(who, None)
            if self._wanted and self._thread is None:
                self._reader = self._make_reader(self._changed.set)
                self._reader.start()
                self._stop.clear()
                self._thread = threading.Thread(target=self._run, name="pilot", daemon=True)
                self._thread.start()
                return
            stop = not self._wanted and self._thread is not None
        if stop:
            self.close()

    @property
    def flying(self) -> bool:
        """Whether a page has this radio fly."""
        return any(self._wanted.values())

    def set_prefs(self, prefs: Mapping[str, Any]) -> None:
        """The player's prefs changed: take their calibrations."""
        self._calibrations = mapping.calibrations(prefs)

    def close(self) -> None:
        self._stop.set()
        self._changed.set()
        thread, reader = self._thread, self._reader
        if thread:
            thread.join(1)
        if reader:
            reader.stop()
        self._thread = self._reader = None

    def _run(self) -> None:
        reader = self._reader
        assert reader is not None
        next_show = 0.0
        while not self._stop.is_set():
            self._changed.wait(REPEAT_S)
            self._changed.clear()
            name, axes = reader.name, reader.axes()
            channels = self._channels(name, axes) if axes is not None else []
            if axes is not None and self.flying:
                self._fly(channels)
            now = time.monotonic()
            if now >= next_show:
                next_show = now + 1 / SHOW_HZ
                self.publish(
                    {"ch": "radio", "name": name, "axes": axes or [], "channels": channels}
                )

    def _channels(self, name: str | None, axes: list[float]) -> list[float]:
        profile = mapping.profile_for(name or "", list(self.content.controllers.values()))
        return mapping.to_channels(axes, [], profile, self._calibrations.get(name or ""))

    def _fly(self, channels: list[float]) -> None:
        vehicle = self.content.vehicles.get(self.session.vehicle or "")
        if vehicle is None or vehicle.firmware.kind == "none":
            return  # the truck: its physics is the page's, and reads the channels there
        self.session.send_rc(mapping.to_firmware(channels, vehicle.firmware.kind))
