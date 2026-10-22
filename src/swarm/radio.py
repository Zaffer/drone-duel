"""USB radio straight into a SITL copter's RC input, with no ground station in between.

Each SITL instance reads RC on UDP 5501 + 10 * instance: packets of 8 or 16 little-endian
uint16 pulse widths, as a receiver would deliver them. `RadioBridge` reads the radio from its
Linux input device (`/dev/input/event*`; on WSL attached with usbipd) and sends one packet per
tick, at ExpressLRS's common 250 Hz, to the copter being flown. Axis n is channel n + 1, the
radio's joystick-mode order (roll, pitch, throttle, yaw, then switches). Pitch is reversed:
ArduPilot reads stick forward as a low pulse.

`RcLink` is the radio links themselves. In the field every drone has a controller of its own,
always linked; the pilot picks up the one of the drone the switch panel selects. In the sim the
USB radio stands in for whichever controller is in the pilot's hands, and every other copter
gets its own controller sitting idle: sticks centred, throttle and switches low. A controller
switched off (`RcLink.power`) is a lost link. SITL keeps the last pulse widths it got forever,
so a copter that simply stops getting packets would never notice; on hardware ArduPilot holds
the last input for RC_FS_TIMEOUT, then enters radio failsafe. So once RC_FS_TIMEOUT_S has
passed, a copter whose controller is off gets failsafe throttle (below FS_THR_VALUE).

Sim only. On hardware the radio talks to each drone's receiver directly.
"""

import fcntl
import logging
import os
import select
import socket
import struct
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

from pymavlink.dialects.v20 import ardupilotmega as apm

log = logging.getLogger(__name__)

SITL_RC_BASE_PORT = 5501
SITL_RC_PORT_STRIDE = 10
CHANNELS = 8
RATE_HZ = 250.0
PWM_MIN = 1000
PWM_MAX = 2000
HOLD_PWM = (1500, 1500, 1500, 1500)
"""Roll, pitch, throttle, yaw sent when the radio is lost or the bridge stops: in LOITER the
copter holds position and height."""
MODE_SWITCH_CHANNEL = 6
"""The radio's 3-position flight-mode switch (axis 5): low LOITER, middle STABILIZE, high ACRO."""
DEFAULT_REVERSED = frozenset({2})
"""Channels whose direction is flipped. Channel 2: the radio reports pitch forward high."""
RC_FS_TIMEOUT_S = 1.0
"""ArduPilot's RC_FS_TIMEOUT: how long a copter holds its last input once the link is gone."""
LOST_THROTTLE_PWM = 900
"""Throttle below FS_THR_VALUE (975): ArduPilot's radio failsafe, as on a lost link."""
LOST_PWM = (1500, 1500, LOST_THROTTLE_PWM, 1500) + (PWM_MIN,) * (CHANNELS - 4)
IDLE_PWM = (1500, 1500, PWM_MIN, 1500) + (PWM_MIN,) * (CHANNELS - 4)
"""A controller on and linked but put down: sticks centred, throttle and switches low."""

EV_ABS = 3
INPUT_EVENT = struct.Struct("llHHi")
EVIOCGABS = 0x80184540
"""_IOR('E', 0x40 + axis, struct input_absinfo); add the axis code."""


def find_radio() -> Path | None:
    """The first joystick in /dev/input/by-id, which is where usbipd-attached radios appear."""
    found = sorted(Path("/dev/input/by-id").glob("*-event-joystick"))
    return found[0] if found else None


def sitl_rc_port(sysid: int) -> int:
    """`swarm sim` starts copter `sysid` as SITL instance sysid - 1."""
    return SITL_RC_BASE_PORT + SITL_RC_PORT_STRIDE * (sysid - 1)


def to_pwm(value: int, lo: int, hi: int, reversed_: bool = False) -> int:
    """Raw axis value in [lo, hi] to a pulse width in [1000, 2000] µs."""
    if hi <= lo:
        return (PWM_MIN + PWM_MAX) // 2
    fraction = min(max((value - lo) / (hi - lo), 0.0), 1.0)
    if reversed_:
        fraction = 1.0 - fraction
    return round(PWM_MIN + fraction * (PWM_MAX - PWM_MIN))


def switch_mode(pwm_us: int) -> int:
    """ArduPilot mode for a MODE_SWITCH_CHANNEL pulse width."""
    if pwm_us < 1300:
        return apm.COPTER_MODE_LOITER
    if pwm_us > 1700:
        return apm.COPTER_MODE_ACRO
    return apm.COPTER_MODE_STABILIZE


def rc_packet(pwm: list[int]) -> bytes:
    return struct.pack(f"<{len(pwm)}H", *pwm)


@dataclass
class Axis:
    lo: int
    hi: int
    value: int


class RcLink:
    """Every copter's controller link to its SITL receiver. `send` once per tick, from one
    thread; `retarget` and `power` from any."""

    def __init__(
        self,
        fleet: list[int],
        sysid: int | None,
        host: str = "127.0.0.1",
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.fleet = fleet
        """Every copter, each with a controller of its own."""
        self.sysid = sysid
        self._host = host
        self._clock = clock
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._off_s: dict[int, float] = {}
        """When each controller switched off was switched off."""

    def retarget(self, sysid: int | None) -> None:
        """The USB radio flies copter `sysid`, or none, from the next packet on. The copter
        left behind gets its own controller back, idle."""
        if sysid == self.sysid:
            return
        log.info("radio -> %s", "no copter" if sysid is None else f"copter {sysid}")
        self.sysid = sysid

    def power(self, sysid: int, on: bool) -> None:
        """Switch copter `sysid`'s controller on or off. Off, its link is lost."""
        if on:
            self._off_s.pop(sysid, None)
        else:
            self._off_s.setdefault(sysid, self._clock())

    def send(self, pwm: list[int]) -> None:
        """`pwm` to the copter flown, idle sticks to every other copter, and failsafe throttle
        to any whose controller has been off for RC_FS_TIMEOUT_S."""
        sysid, now = self.sysid, self._clock()
        copters = self.fleet if sysid is None or sysid in self.fleet else [*self.fleet, sysid]
        for copter in copters:
            off_s = self._off_s.get(copter)
            if off_s is None:
                self._send_to(copter, pwm if copter == sysid else list(IDLE_PWM))
            elif now - off_s >= RC_FS_TIMEOUT_S:
                self._send_to(copter, list(LOST_PWM))
            # else: no packets; SITL holds the last input, as ArduPilot does until the timeout

    def _send_to(self, sysid: int, pwm: list[int]) -> None:
        try:
            self._sock.sendto(rc_packet(pwm), (self._host, sitl_rc_port(sysid)))
        except OSError as exc:
            log.debug("rc send failed: %s", exc)


class RadioBridge:
    """Reads the radio in a thread and streams its channels to one SITL copter."""

    def __init__(
        self,
        device: Path,
        sysid: int | None,
        fleet: list[int] | None = None,
        reversed_channels: frozenset[int] = DEFAULT_REVERSED,
        host: str = "127.0.0.1",
    ) -> None:
        self.device = device
        self.reversed_channels = reversed_channels
        self._link = RcLink(fleet or ([] if sysid is None else [sysid]), sysid, host)
        self._axes: dict[int, Axis] = {}
        self._connected = False
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="radio", daemon=True)

    @property
    def sysid(self) -> int | None:
        return self._link.sysid

    def start(self) -> None:
        target = "no copter" if self.sysid is None else f"copter {self.sysid}"
        log.info("radio %s -> %s", self.device, target)
        self._thread.start()

    def retarget(self, sysid: int | None) -> None:
        """Fly copter `sysid`, or none (`RcLink.retarget`)."""
        self._link.retarget(sysid)

    def mode(self) -> int:
        """The mode the flight-mode switch asks for."""
        return switch_mode(self.channels()[MODE_SWITCH_CHANNEL - 1])

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2)
        self._send(self._hold())

    def channels(self) -> list[int]:
        """Current pulse widths, channel 1 first."""
        with self._lock:
            if not self._connected:
                return self._hold()
            pwm: list[int] = []
            for code in range(CHANNELS):
                axis = self._axes.get(code)
                if axis is None:
                    pwm.append(PWM_MIN)
                else:
                    flip = (code + 1) in self.reversed_channels
                    pwm.append(to_pwm(axis.value, axis.lo, axis.hi, flip))
            return pwm

    def _hold(self) -> list[int]:
        """Centred sticks; switches keep their last positions."""
        with_switches = [PWM_MIN] * CHANNELS
        for code in range(4, CHANNELS):
            axis = self._axes.get(code)
            if axis is not None:
                with_switches[code] = to_pwm(axis.value, axis.lo, axis.hi)
        with_switches[:4] = HOLD_PWM
        return with_switches

    def _send(self, pwm: list[int]) -> None:
        self._link.send(pwm)

    def _run(self) -> None:
        period = 1.0 / RATE_HZ
        fd: int | None = None
        next_send = time.monotonic()
        while not self._stop.is_set():
            if fd is None:
                fd = self._open()
                if fd is None:
                    self._send(self.channels())
                    self._stop.wait(1.0)
                    continue
            timeout = max(0.0, next_send - time.monotonic())
            try:
                ready, _, _ = select.select([fd], [], [], timeout)
                if ready:
                    self._read(fd)
            except OSError as exc:
                log.warning("radio lost (%s); holding: sticks centred", exc)
                os.close(fd)
                fd = None
                with self._lock:
                    self._connected = False
            if time.monotonic() >= next_send:
                self._send(self.channels())
                next_send += period
                if next_send < time.monotonic():
                    next_send = time.monotonic() + period
        if fd is not None:
            os.close(fd)

    def _open(self) -> int | None:
        try:
            fd = os.open(self.device, os.O_RDONLY | os.O_NONBLOCK)
        except OSError as exc:
            log.warning("cannot open %s (%s); retrying", self.device, exc)
            return None
        axes = read_axes(fd)
        with self._lock:
            self._axes = axes
            self._connected = True
        log.info("radio open: %d axes", len(axes))
        return fd

    def _read(self, fd: int) -> None:
        data = os.read(fd, INPUT_EVENT.size * 64)
        with self._lock:
            apply_events(data, self._axes)


class RadioReader:
    """The radio's channels for a web page, the viewer's truck (`swarm.view`'s /radio): each
    -1..1 as the radio reports it, nothing reversed. Chrome's Gamepad API on Linux misses
    most of this radio's axes; evdev has them all. Reads in a thread of its own, sharing the
    device with `RadioBridge` (evdev hands every reader every event), and finds the radio again
    once it is plugged back in."""

    def __init__(self, find: Callable[[], Path | None] = find_radio) -> None:
        self._find = find
        self._axes: dict[int, Axis] = {}
        self._connected = False
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="radio-reader", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2)

    def channels(self) -> list[float] | None:
        """Channel 1 first, each -1..1; None with no radio."""
        with self._lock:
            if not self._connected:
                return None
            axes = self._axes
            return [unit(axes[code]) if code in axes else -1.0 for code in range(CHANNELS)]

    def _run(self) -> None:
        while not self._stop.is_set():
            device = self._find()
            try:
                fd = os.open(device, os.O_RDONLY | os.O_NONBLOCK) if device else None
            except OSError as exc:
                log.debug("cannot open %s (%s)", device, exc)
                fd = None
            if fd is None:
                self._stop.wait(1.0)
                continue
            with self._lock:
                self._axes = read_axes(fd)
                self._connected = True
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
                    self._connected = False


def read_axes(fd: int) -> dict[int, Axis]:
    """Each channel's range and value now, from the open input device `fd`."""
    axes: dict[int, Axis] = {}
    for code in range(CHANNELS):
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
