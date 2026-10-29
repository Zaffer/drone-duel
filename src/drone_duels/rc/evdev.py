"""The radio read through Linux itself (evdev), for when Chrome's Gamepad API cannot see all of it.

A drone radio in USB joystick mode is a HID joystick: on Linux an input device with up to eight
absolute axes (X, Y, Z, RX, RY, RZ, THROTTLE, RUDDER), channel n on axis n. `RadioReader` finds the
first joystick in /proc/bus/input/devices, reads it in a thread of its own and finds it again once
it is plugged back in. The page asks for it (protocol.RadioSourceMsg) and gets every axis, -1..1
as the radio reports it, nothing reversed; it maps and calibrates them as it does a gamepad's.
"""

import fcntl
import logging
import os
import re
import select
import struct
import threading
from dataclasses import dataclass
from pathlib import Path

log = logging.getLogger(__name__)

AXES = 8
EV_ABS = 3
INPUT_EVENT = struct.Struct("llHHi")
EVIOCGABS = 0x80184540
"""_IOR('E', 0x40 + axis, struct input_absinfo); add the axis code."""
DEVICES = Path("/proc/bus/input/devices")


@dataclass
class Axis:
    lo: int
    hi: int
    value: int


@dataclass(frozen=True)
class Joystick:
    name: str
    event: Path
    vendor: str
    product: str


def find_joysticks(devices: str | None = None) -> list[Joystick]:
    """Every joystick (a `js` handler) with at least four absolute axes."""
    if devices is None:
        try:
            devices = DEVICES.read_text()
        except OSError:
            return []
    found: list[Joystick] = []
    for block in devices.strip().split("\n\n"):
        name = re.search(r'^N: Name="(.*)"$', block, re.M)
        handlers = re.search(r"^H: Handlers=(.*)$", block, re.M)
        ids = re.search(r"^I: .*Vendor=(\w+) Product=(\w+)", block, re.M)
        abs_bits = re.search(r"^B: ABS=(\w+)", block, re.M)
        if not (name and handlers and abs_bits):
            continue
        words = handlers.group(1).split()
        event = next((w for w in words if w.startswith("event")), None)
        if event is None or not any(w.startswith("js") for w in words):
            continue
        if bin(int(abs_bits.group(1).split()[-1], 16)).count("1") < 4:
            continue
        found.append(
            Joystick(
                name=name.group(1),
                event=Path("/dev/input") / event,
                vendor=ids.group(1) if ids else "",
                product=ids.group(2) if ids else "",
            )
        )
    return found


class RadioReader:
    def __init__(self) -> None:
        self._axes: dict[int, Axis] = {}
        self._device: Joystick | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="radio-evdev", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2)

    @property
    def name(self) -> str | None:
        """The radio as Chrome would name it (`Name (Vendor: v Product: p)`), or None."""
        d = self._device
        return f"{d.name} (Vendor: {d.vendor} Product: {d.product})" if d else None

    def axes(self) -> list[float] | None:
        """Every axis, -1..1, axis 0 first; None with no radio."""
        with self._lock:
            if self._device is None:
                return None
            return [unit(self._axes[c]) if c in self._axes else 0.0 for c in range(AXES)]

    def _run(self) -> None:
        while not self._stop.is_set():
            sticks = find_joysticks()
            fd = None
            if sticks:
                try:
                    fd = os.open(sticks[0].event, os.O_RDONLY | os.O_NONBLOCK)
                except OSError as exc:
                    log.info("cannot open %s (%s)", sticks[0].event, exc)
            if fd is None:
                self._stop.wait(1.0)
                continue
            with self._lock:
                self._axes = read_axes(fd)
                self._device = sticks[0]
            log.info("radio: %s", sticks[0].name)
            try:
                while not self._stop.is_set():
                    if select.select([fd], [], [], 0.5)[0]:
                        data = os.read(fd, INPUT_EVENT.size * 64)
                        with self._lock:
                            apply_events(data, self._axes)
            except OSError as exc:
                log.info("radio unplugged (%s)", exc)
            finally:
                os.close(fd)
                with self._lock:
                    self._device = None


def read_axes(fd: int) -> dict[int, Axis]:
    """Each axis's range and value now, from the open input device `fd`."""
    axes: dict[int, Axis] = {}
    for code in range(AXES):
        try:
            raw = fcntl.ioctl(fd, EVIOCGABS + code, bytes(24))
        except OSError:
            continue
        value, lo, hi = struct.unpack_from("3i", raw)
        axes[code] = Axis(lo, hi, value)
    return axes


def apply_events(data: bytes, axes: dict[int, Axis]) -> None:
    """Move `axes` by the input events in `data`; any other event is skipped."""
    for offset in range(0, len(data) - INPUT_EVENT.size + 1, INPUT_EVENT.size):
        _, _, kind, code, value = INPUT_EVENT.unpack_from(data, offset)
        axis = axes.get(code)
        if kind == EV_ABS and axis is not None:
            axis.value = value


def unit(axis: Axis) -> float:
    """`axis` from -1 at its low end to 1 at its high end, to 3 places."""
    if axis.hi <= axis.lo:
        return 0.0
    fraction = min(max((axis.value - axis.lo) / (axis.hi - axis.lo), 0.0), 1.0)
    return round(fraction * 2 - 1, 3)
