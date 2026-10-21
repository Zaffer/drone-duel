"""Logitech (Saitek) Pro Flight Switch Panel as the mission console of `swarm pilot`.

The panel is a USB HID device, 06a3:0d67. Every change of a switch sends one 3-byte input
report holding all 20 contacts, contact n in bit n. A 1-byte feature report lights the three
landing-gear indicators, each a green and a red LED (both on is amber); there is no RGB. On WSL
attach it with usbipd like the radio. It shows up as /dev/hidraw*, found by its USB id.

  gear lever   the safety switch. DOWN lands every armed drone and locks the panel. UP
               unlocks it (live), but only after the lever has been seen DOWN, so a lever
               already UP at start arms nothing.
  ALT          the leader: ON arms it, ready, holding on the ground until the radio takes it.
  AVIONICS, FUEL PUMP, DE-ICE, PITOT HEAT
               followers a-d: ON arms and launches on the mission.
               OFF lands that drone where it is; ON again calls the landing off and it
               takes up the mission again.
  rotary       mission phase, flown START, BOTH, L, then R or OFF: START launch and hold
               where they are, BOTH follow the leader, L hold where they are (GUIDED), R
               everything home, OFF land everything in the air (drones still arm on the
               ground and wait for START). BOTH back to START or L pauses the swarm where it
               is; BOTH again and it rejoins. A position counts once the rotary has rested
               there ROTARY_SETTLE_S, so turning past L and R to OFF sends nothing home.
  bottom row   the radio's drone selector: BEACON the leader, NAV/STROBE/TAXI/LANDING
               followers a-d. COWL is auto select: the leader first, then, whenever the drone
               flown is disarmed, the next drone in the air. The drone selected goes to LOITER
               under the sticks and its mission waits; the radio's mode switch then picks
               LOITER, STABILIZE or ACRO once it has been at LOITER. None on, the radio flies
               nothing and the leader holds where it is. Exactly one switch may be on: two or
               more select nothing until every one is off again.
  BAT, PANEL   nothing.

Everything but the gear acts only while live. A switch already ON when the panel goes live
acts as it goes live.

Gear indicators: nose red locked, amber live with no drone on the radio (none selected, two
selected, or the mode switch not yet back at LOITER), green live with the radio flying a drone.
Left is the leader and right the followers: off nothing armed, amber arming or on the ground,
green flying, red landing or returning home.
"""

import fcntl
import logging
import os
import time
from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import TYPE_CHECKING

from swarm.launch import KEYS, Follower, Launcher, Phase, Stage

if TYPE_CHECKING:
    from swarm.state import VehicleState

log = logging.getLogger(__name__)

VENDOR_ID = 0x06A3
PRODUCT_ID = 0x0D67
REPORT_SIZE = 3
REOPEN_S = 1.0

# Contact bits. 0-5 and 13-19 checked on the panel; 6-12 from its layout.
BAT, ALT, AVIONICS, FUEL_PUMP, DE_ICE, PITOT_HEAT = range(6)
COWL, PANEL, BEACON, NAV, STROBE, TAXI, LANDING = range(6, 13)
MAG_OFF, MAG_R, MAG_L, MAG_BOTH, MAG_START = range(13, 18)
GEAR_UP, GEAR_DOWN = 18, 19

SWITCH_NAMES = {
    BAT: "BAT",
    ALT: "ALT",
    AVIONICS: "AVIONICS",
    FUEL_PUMP: "FUEL PUMP",
    DE_ICE: "DE-ICE",
    PITOT_HEAT: "PITOT HEAT",
    COWL: "COWL",
    PANEL: "PANEL",
    BEACON: "BEACON",
    NAV: "NAV",
    STROBE: "STROBE",
    TAXI: "TAXI",
    LANDING: "LANDING",
}
ROTARY = {
    MAG_OFF: ("OFF", Phase.LAND),
    MAG_R: ("R", Phase.RTL),
    MAG_L: ("L", Phase.HOLD),
    MAG_BOTH: ("BOTH", Phase.FOLLOW),
    MAG_START: ("START", Phase.HOLD),
}
FOLLOWER_SWITCHES = dict(zip((AVIONICS, FUEL_PUMP, DE_ICE, PITOT_HEAT), KEYS, strict=True))
"""Follower switch -> the launcher key of the follower it flies."""
AUTO, LEADER = "auto", "leader"
TAKEOVER_SWITCHES = {
    COWL: AUTO,
    BEACON: LEADER,
    **dict(zip((NAV, STROBE, TAXI, LANDING), KEYS, strict=True)),
}
"""Bottom row -> the drone it hands the radio: auto select, the leader or a follower's key."""
ROTARY_SETTLE_S = 0.75
"""The rotary must rest this long in a position for its phase to start."""

HIDIOCSFEATURE = 0x06
HIDIOCGINPUT = 0x0A


def _hidioc(nr: int, size: int) -> int:
    """_IOC(_IOC_READ | _IOC_WRITE, 'H', nr, size), as in linux/hidraw.h."""
    return (3 << 30) | (size << 16) | (ord("H") << 8) | nr


class Led(Enum):
    OFF = 0
    GREEN = 1
    RED = 2
    AMBER = 3


def led_report(nose: Led, left: Led, right: Led) -> int:
    """Feature byte: green nose, left, right in bits 0-2, red in bits 3-5."""
    value = 0
    for gear, led in enumerate((nose, left, right)):
        if led.value & Led.GREEN.value:
            value |= 1 << gear
        if led.value & Led.RED.value:
            value |= 1 << (gear + 3)
    return value


def leds_from_report(value: int) -> tuple[Led, Led, Led]:
    """The nose, left and right LEDs a feature byte lights (`led_report` undone)."""
    return tuple(  # pyright: ignore[reportReturnType]
        Led(int(bool(value >> gear & 1)) | int(bool(value >> (gear + 3) & 1)) << 1)
        for gear in range(3)
    )


@dataclass(frozen=True)
class PanelState:
    bits: int

    @classmethod
    def from_report(cls, report: bytes) -> "PanelState":
        return cls(int.from_bytes(report[:REPORT_SIZE], "little"))

    def on(self, contact: int) -> bool:
        return bool(self.bits >> contact & 1)

    @property
    def gear_down(self) -> bool | None:
        """None while the lever is between its positions."""
        if self.on(GEAR_DOWN):
            return True
        if self.on(GEAR_UP):
            return False
        return None

    @property
    def phase(self) -> Phase | None:
        """The rotary's phase; None between positions."""
        for contact, (_, phase) in ROTARY.items():
            if self.on(contact):
                return phase
        return None

    def describe(self) -> str:
        switches = [name for contact, name in SWITCH_NAMES.items() if self.on(contact)]
        rotary = next((name for c, (name, _) in ROTARY.items() if self.on(c)), "-")
        gear = {True: "DOWN", False: "UP", None: "-"}[self.gear_down]
        return f"gear {gear}, rotary {rotary}, on: {', '.join(switches) or 'none'}"


def find_panel(sys_hidraw: Path = Path("/sys/class/hidraw")) -> Path | None:
    hid_id = f"HID_ID=0003:{VENDOR_ID:08X}:{PRODUCT_ID:08X}"
    for node in sorted(sys_hidraw.glob("hidraw*")):
        try:
            uevent = (node / "device" / "uevent").read_text()
        except OSError:
            continue
        if hid_id in uevent.split():
            return Path("/dev") / node.name
    return None


class SwitchPanel:
    """The panel's hidraw device, polled without blocking. Found again if it goes away."""

    def __init__(self) -> None:
        self._fd: int | None = None
        self._next_open_s = 0.0
        self._warned = False
        self._initial: PanelState | None = None
        self._leds: int | None = None
        self.state: PanelState | None = None

    @property
    def attached(self) -> bool:
        return self._fd is not None

    def poll(self) -> PanelState | None:
        """The switch positions if they changed since the last call, else None."""
        if self._fd is None and not self._open():
            return None
        assert self._fd is not None
        latest, self._initial = self._initial, None
        try:
            while True:
                report = os.read(self._fd, 64)
                if len(report) >= REPORT_SIZE:
                    latest = PanelState.from_report(report)
        except BlockingIOError:
            pass
        except OSError as exc:
            log.warning("switch panel lost (%s); switches hold their last positions", exc)
            self._close()
            return None
        if latest is None or latest == self.state:
            return None
        self.state = latest
        return latest

    def set_leds(self, value: int) -> None:
        if self._fd is None or value == self._leds:
            return
        try:
            fcntl.ioctl(self._fd, _hidioc(HIDIOCSFEATURE, 2), bytes([0, value]))
            self._leds = value
        except OSError as exc:
            log.debug("panel LEDs: %s", exc)

    def close(self) -> None:
        self.set_leds(0)
        self._close()

    def _open(self) -> bool:
        now = time.monotonic()
        if now < self._next_open_s:
            return False
        self._next_open_s = now + REOPEN_S
        device = find_panel()
        if device is None:
            return False
        try:
            fd = os.open(device, os.O_RDWR | os.O_NONBLOCK)
        except OSError as exc:
            if not self._warned:
                log.warning("cannot open switch panel %s (%s); see README", device, exc)
                self._warned = True
            return False
        report = bytearray(REPORT_SIZE + 1)  # report number 0, then the report
        try:
            fcntl.ioctl(fd, _hidioc(HIDIOCGINPUT, len(report)), report, True)
            self._initial = PanelState.from_report(bytes(report[1:]))
        except OSError:
            self._initial = None  # positions arrive with the first change
        log.info("switch panel %s open", device)
        self._fd, self._leds, self._warned = fd, None, False
        return True

    def _close(self) -> None:
        if self._fd is not None:
            os.close(self._fd)
        self._fd = None
        self._leds = None


class PanelControl:
    """Turns switch positions into launcher commands, and launcher state into gear LEDs."""

    def __init__(
        self,
        launcher: Launcher,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.launcher = launcher
        self.clock = clock
        launcher.live = False
        launcher.phase = Phase.NONE
        self.unlocked = False
        self._gear_down: bool | None = None
        self._rotary: Phase | None = None
        self._rotary_since_s = 0.0
        self._on: dict[int, bool] = {}
        """Per drone switch: ON while live, at the last update."""
        self._selection: str | None = None
        """What the takeover row asks for: AUTO, LEADER, a follower's key, or None."""
        self._blocked = False
        """Two or more takeover switches were on; cleared when all are off."""
        self._auto: int = launcher.leader
        """The copter auto select flies."""

    @property
    def live(self) -> bool:
        return self.unlocked

    def update(self, state: PanelState) -> None:
        was_live = self.live
        self._gear(state.gear_down)
        if self.live and not was_live:
            self.launcher.unlock()
        if state.phase is not None and state.phase is not self._rotary:
            self._rotary, self._rotary_since_s = state.phase, self.clock()
        edge = self._edge(state, ALT)
        if edge is True:
            self.launcher.arm_leader()
        elif edge is False:
            self.launcher.land_leader()
        for contact, key in FOLLOWER_SWITCHES.items():
            edge = self._edge(state, contact)
            if key not in self.launcher.followers:
                continue
            if edge is True:
                self.launcher.launch(key)
            elif edge is False:
                self.launcher.land(key)
        self._takeover_row(state)

    def step(self) -> None:
        """Start the rotary's phase once it has rested there, and hand the radio to the copter
        the bottom row selects."""
        rested = self.clock() - self._rotary_since_s >= ROTARY_SETTLE_S
        if self.live and self._rotary is not None and rested:
            self.launcher.set_phase(self._rotary)
        self.launcher.take_over(self._target() if self.live else None)

    def leds(self) -> int:
        flying = self.launcher.manual is not None and self.launcher.mode_cleared
        nose = (Led.GREEN if flying else Led.AMBER) if self.live else Led.RED
        followers = list(self.launcher.followers.values())
        return led_report(nose, self._colour([self.launcher.lead]), self._colour(followers))

    def _takeover_row(self, state: PanelState) -> None:
        on = [contact for contact in TAKEOVER_SWITCHES if state.on(contact)]
        if len(on) > 1 and not self._blocked:
            names = ", ".join(SWITCH_NAMES[c] for c in on)
            log.warning("selector switches %s on at once: radio off; all off, then one", names)
            self._blocked = True
        elif not on:
            self._blocked = False
        selection = TAKEOVER_SWITCHES[on[0]] if len(on) == 1 and not self._blocked else None
        if selection == self._selection:
            return
        self._selection = selection
        if selection == AUTO:
            self._auto = self.launcher.leader
            log.info("auto select: radio to the leader, then the next drone in the air")

    def _target(self) -> int | None:
        if self._selection is None:
            return None
        if self._selection == AUTO:
            return self._auto_target()
        if self._selection == LEADER:
            return self.launcher.leader
        f = self.launcher.followers.get(self._selection)
        return None if f is None else f.sysid

    def _auto_target(self) -> int:
        """The copter auto select flies, moved on to the next drone in the air once it has
        disarmed. It stays put when no other drone is in the air."""
        vehicles = self.launcher.link.vehicles
        current = vehicles.get(self._auto)
        if current is not None and current.armed:
            return self._auto
        order = [self.launcher.leader, *(f.sysid for f in self.launcher.followers.values())]
        k = order.index(self._auto) if self._auto in order else 0
        for sysid in order[k + 1 :] + order[:k]:
            v = vehicles.get(sysid)
            if v is not None and v.armed and v.in_air:
                log.info("auto select: copter %d disarmed, radio to copter %d", self._auto, sysid)
                self._auto = sysid
                break
        return self._auto

    def _gear(self, down: bool | None) -> None:
        if down is None or down == self._gear_down:
            return
        if down:
            if self._gear_down is False:
                log.warning("gear DOWN: every drone lands; panel locked")
                self.launcher.lock()
            else:
                log.info("gear DOWN: panel locked; gear UP unlocks it")
            self.unlocked = False
        elif self._gear_down:
            log.info("gear UP: panel live")
            self.unlocked = True
        else:
            log.warning("gear is UP: put it DOWN and UP again to unlock the panel")
        self._gear_down = down

    def _edge(self, state: PanelState, contact: int) -> bool | None:
        """True when a drone switch should arm its drone, False when it should land it.

        A switch counts as ON only while live. Going live with it ON arms; gear DOWN is the
        launcher's lock, not a landing per switch."""
        on = self.live and state.on(contact)
        was = self._on.get(contact, False)
        self._on[contact] = on
        if on and not was:
            return True
        if was and not on and self.live:
            return False
        return None

    def _colour(self, crafts: list[Follower]) -> Led:
        colours = {self._craft_colour(f, self.launcher.link.vehicles.get(f.sysid)) for f in crafts}
        return next((c for c in (Led.RED, Led.AMBER, Led.GREEN) if c in colours), Led.OFF)

    @staticmethod
    def _craft_colour(f: Follower, v: "VehicleState | None") -> Led:
        if f.stage in (Stage.LANDING, Stage.RETURNING):
            return Led.RED
        if v is None:
            return Led.OFF
        if v.armed and v.in_air and f.stage is not Stage.LIFTOFF:
            return Led.GREEN
        if v.armed or f.stage is not Stage.IDLE:
            return Led.AMBER
        return Led.OFF
