"""`swarm rehearse`: the whole mission, hands-free, on the sim stack.

The switch panel and the radio are scripted stand-ins. Everything behind them is the code that
flies the real thing: the console, the launcher and the panel rules, MAVLink through
mavlink-router to ArduPilot SITL, the radio link with its failsafe (`swarm.radio.RcLink`), and
the watcher feeding the 3D viewer (the COP). Each step does what the pilot would and waits, up
to a timeout, for what ArduPilot and the COP report. The run stops at the first step that
fails and says what was missing.
"""

import json
import logging
import math
import queue
import threading
import time
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

import numpy as np
from pymavlink.dialects.v20 import ardupilotmega as apm

from swarm.console import Console
from swarm.follow import follow_slot_ned
from swarm.geo import LocalFrame
from swarm.launch import (
    AVOID_ESCAPE,
    AVOID_REPORT,
    KEYS,
    RELEASE_HOLD_S,
    Follower,
    Launcher,
    Stage,
)
from swarm.panel import (
    ALT,
    BEACON,
    COWL,
    FOLLOWER_SWITCHES,
    GEAR_DOWN,
    GEAR_UP,
    MAG_BOTH,
    MAG_L,
    MAG_OFF,
    MAG_R,
    MAG_START,
    SWITCH_NAMES,
    TAKEOVER_SWITCHES,
    Led,
    PanelState,
)
from swarm.radio import CHANNELS, MODE_SWITCH_CHANNEL, PWM_MIN, RATE_HZ, RcLink, switch_mode
from swarm.state import mode_name

if TYPE_CHECKING:
    from swarm.link import MavlinkLink
    from swarm.sim import SimField
    from swarm.state import VehicleState
    from swarm.view import ViewServer

log = logging.getLogger(__name__)

TICK_S = 0.05
SLOT_TOLERANCE_M = 3.0
"""A follower is in its slot this close to where ArduPilot FOLLOW puts it."""
FORWARD_PWM = 1300
"""Pitch stick part forward: ArduPilot reads forward as a low pulse."""
CLIMB_PWM = 1800
CLIMB_STEP_PWM = 200
"""Throttle off centre for a slow climb or descent in LOITER, beyond its dead zone."""
B_RING_M = 8.0
"""Each drone lands on its own spot this far from B, inside mission.toml's landing_zone_radius_m."""
MODE_SWITCH_PWM = {apm.COPTER_MODE_LOITER: 1000, apm.COPTER_MODE_STABILIZE: 1500}


class ScriptedPanel:
    """The switch panel as the script sets it. `poll` reports a change once, like the panel."""

    def __init__(self) -> None:
        self.gear_down = True
        self.rotary = MAG_OFF
        self.on: set[int] = set()
        self.leds = 0
        self._sent: int | None = None

    def poll(self) -> PanelState | None:
        bits = sum(1 << c for c in self.on) | 1 << self.rotary
        bits |= 1 << (GEAR_DOWN if self.gear_down else GEAR_UP)
        if bits == self._sent:
            return None
        self._sent = bits
        return PanelState(bits)

    def set_leds(self, value: int) -> None:
        self.leds = value

    def nose(self) -> Led:
        green, red = bool(self.leds & 1), bool(self.leds & 1 << 3)
        return Led(int(green) | int(red) << 1)


class ScriptedRadio:
    """The radio as the script moves it, streamed at RATE_HZ over the sim radio link."""

    def __init__(self, fleet: list[int]) -> None:
        self.pwm = [1500, 1500, PWM_MIN, 1500] + [PWM_MIN] * (CHANNELS - 4)
        self._link = RcLink(fleet, None)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="scripted-radio", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2)

    def sticks(
        self, roll: int = 1500, pitch: int = 1500, throttle: int | None = None, yaw: int = 1500
    ) -> None:
        """Roll, pitch and yaw centred unless given; throttle stays where it is unless given."""
        self.pwm[0], self.pwm[1], self.pwm[3] = roll, pitch, yaw
        if throttle is not None:
            self.pwm[2] = throttle

    def mode_switch(self, mode: int) -> None:
        self.pwm[MODE_SWITCH_CHANNEL - 1] = MODE_SWITCH_PWM[mode]

    def mode(self) -> int:
        return switch_mode(self.pwm[MODE_SWITCH_CHANNEL - 1])

    def retarget(self, sysid: int | None) -> None:
        self._link.retarget(sysid)

    def power(self, sysid: int, on: bool) -> None:
        """Switch copter `sysid`'s own controller on or off."""
        self._link.power(sysid, on)

    def _run(self) -> None:
        while not self._stop.wait(1.0 / RATE_HZ):
            self._link.send(list(self.pwm))


class StepError(RuntimeError):
    pass


@dataclass
class Result:
    name: str
    ok: bool
    seconds: float
    note: str = ""


Check = Callable[[], str]
"""Empty when satisfied, else what is still missing."""


def all_of(*checks: Check) -> Check:
    def check() -> str:
        return next((m for c in checks if (m := c())), "")

    return check


class Rehearsal:
    def __init__(
        self,
        console: Console,
        panel: ScriptedPanel,
        radio: ScriptedRadio,
        viewer: "ViewServer | None",
    ) -> None:
        self.console = console
        self.launcher: Launcher = console.launcher
        self.link: MavlinkLink = console.launcher.link
        self.panel = panel
        self.radio = radio
        self.results: list[Result] = []
        self.cop: dict[str, Any] | None = None
        """The latest frame the 3D viewer sent its pages."""
        self._cop_events = viewer.hub.subscribe()[0] if viewer is not None else None
        self._note = ""

    # -- running -------------------------------------------------------------------

    def tick(self) -> None:
        self.console.tick()
        while self._cop_events is not None:
            try:
                kind, data = self._cop_events.get_nowait()
            except queue.Empty:
                break
            if kind == "frame":
                self.cop = json.loads(data)

    def wait(
        self, check: Check, timeout_s: float, during: Callable[[], None] | None = None
    ) -> None:
        """Tick until `check` is satisfied, calling `during` each tick."""
        deadline = time.monotonic() + timeout_s
        while True:
            start = time.monotonic()
            self.tick()
            if during is not None:
                during()
            missing = check()
            if not missing:
                return
            if start > deadline:
                raise StepError(f"after {timeout_s:.0f} s still waiting for {missing}")
            time.sleep(max(0.0, TICK_S - (time.monotonic() - start)))

    def keep(self, check: Check, seconds: float, during: Callable[[], None] | None = None) -> None:
        """Tick for `seconds`; `check` must hold all along."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            start = time.monotonic()
            self.tick()
            if during is not None:
                during()
            missing = check()
            if missing:
                raise StepError(f"lost {missing}")
            time.sleep(max(0.0, TICK_S - (time.monotonic() - start)))

    def run(self, seconds: float, during: Callable[[], None] | None = None) -> None:
        self.keep(lambda: "", seconds, during)

    def note(self, text: str) -> None:
        self._note = text

    @contextmanager
    def step(self, name: str) -> Iterator[None]:
        log.info("STEP %s", name)
        self._note = ""
        start = time.monotonic()
        try:
            yield
        except StepError as exc:
            self.results.append(Result(name, False, time.monotonic() - start, str(exc)))
            log.error("FAIL %s: %s", name, exc)
            for line in self.fleet():
                log.error("  %s", line)
            raise
        self.results.append(Result(name, True, time.monotonic() - start, self._note))
        log.info("PASS %s%s", name, f" ({self._note})" if self._note else "")

    def fleet(self) -> list[str]:
        lines = []
        for f in self._crafts():
            v = self.link.vehicles.get(f.sysid)
            if v is None:
                lines.append(f"copter {f.sysid}: not connected")
                continue
            air = f"in the air {v.rel_alt_m:.1f} m" if v.in_air else "on the ground"
            arm = "armed" if v.armed else "disarmed"
            radio = ", radio" if self.launcher.manual == f.sysid else ""
            lines.append(
                f"copter {f.sysid}: {mode_name(v.mode)} {arm} {air}, "
                f"launcher {f.stage.value}{radio}"
            )
        return lines

    # -- what ArduPilot reports --------------------------------------------------------

    def _crafts(self) -> list[Follower]:
        return [self.launcher.lead, *self.launcher.followers.values()]

    def vehicle(self, sysid: int) -> "VehicleState":
        return self.link.vehicles[sysid]

    def ready(self, count: int) -> Check:
        def check() -> str:
            vs = self.link.vehicles
            if len(vs) < count:
                return f"{count} copters (have {sorted(vs)})"
            for v in vs.values():
                if not v.position_ok:
                    return f"copter {v.sysid} EKF position"
                if not v.prearm_ok:
                    return f"copter {v.sysid} pre-arm checks"
            return ""

        return check

    def mode(self, sysid: int, mode: int) -> Check:
        def check() -> str:
            v = self.vehicle(sysid)
            if v.mode == mode:
                return ""
            return f"copter {sysid} in {mode_name(mode)} (is {mode_name(v.mode)})"

        return check

    def armed(self, sysid: int) -> Check:
        return lambda: "" if self.vehicle(sysid).armed else f"copter {sysid} armed"

    def disarmed(self, sysid: int) -> Check:
        return lambda: (
            "" if not self.vehicle(sysid).armed else f"copter {sysid} landed and disarmed"
        )

    def rc(self, sysid: int, ok: bool) -> Check:
        def check() -> str:
            if self.vehicle(sysid).rc_ok == ok:
                return ""
            return f"copter {sysid} {'with its RC link' if ok else 'in radio failsafe'}"

        return check

    def prearm(self, sysid: int, ok: bool) -> Check:
        def check() -> str:
            if self.vehicle(sysid).prearm_ok == ok:
                return ""
            return f"copter {sysid} pre-arm checks {'passing' if ok else 'failing'}"

        return check

    def param(self, sysid: int, name: str, value: float) -> Check:
        def check() -> str:
            got = self.vehicle(sysid).params.get(name)
            return "" if got == value else f"copter {sysid} {name} {value:g} (is {got})"

        return check

    def separation_m(self, a: int, b: int) -> float:
        va = self.vehicle(a)
        return float(
            np.linalg.norm(
                self.ned(b, LocalFrame(va.lat_deg, va.lon_deg))
                - self.ned(a, LocalFrame(va.lat_deg, va.lon_deg))
            )
        )

    def in_air(self, sysid: int) -> Check:
        return lambda: "" if self.vehicle(sysid).in_air else f"copter {sysid} in the air"

    def on_ground(self, sysid: int) -> Check:
        return lambda: "" if not self.vehicle(sysid).in_air else f"copter {sysid} on the ground"

    def above(self, sysid: int, alt_m: float) -> Check:
        def check() -> str:
            alt = self.vehicle(sysid).rel_alt_m
            return "" if alt >= alt_m else f"copter {sysid} above {alt_m:.0f} m (at {alt:.1f} m)"

        return check

    def stage(self, sysid: int, stage: Stage) -> Check:
        def check() -> str:
            f = next(f for f in self._crafts() if f.sysid == sysid)
            if f.stage is stage:
                return ""
            return f"copter {sysid} launcher {stage.value} (is {f.stage.value})"

        return check

    def radio_on(self, sysid: int | None) -> Check:
        def check() -> str:
            if self.launcher.manual == sysid:
                return ""
            want = "no copter" if sysid is None else f"copter {sysid}"
            return f"the radio on {want} (on {self.launcher.manual})"

        return check

    def live(self, live: bool) -> Check:
        return lambda: "" if self.launcher.live == live else f"panel {'live' if live else 'locked'}"

    def nose(self, led: Led) -> Check:
        def check() -> str:
            got = self.panel.nose()
            return "" if got is led else f"nose LED {led.name} (is {got.name})"

        return check

    def ned(self, sysid: int, frame: LocalFrame) -> np.ndarray:
        v = self.vehicle(sysid)
        n, e = frame.to_ne(v.lat_deg, v.lon_deg)
        return np.array([n, e, -v.rel_alt_m])

    def slot_error_m(self, sysid: int) -> float | None:
        """Distance from where ArduPilot FOLLOW puts `sysid` behind the leader, or None."""
        lead = self.vehicle(self.launcher.leader)
        frame = LocalFrame(lead.lat_deg, lead.lon_deg)
        slot = follow_slot_ned(self.vehicle(sysid), lead, np.array([0.0, 0.0, -lead.rel_alt_m]))
        if slot is None:
            return None
        return float(np.linalg.norm(self.ned(sysid, frame) - slot))

    def in_slot(self, sysid: int, tolerance_m: float = SLOT_TOLERANCE_M) -> Check:
        def check() -> str:
            error = self.slot_error_m(sysid)
            if error is not None and error <= tolerance_m:
                return ""
            at = "slot unknown" if error is None else f"{error:.1f} m off"
            return f"copter {sysid} within {tolerance_m:.0f} m of its slot ({at})"

        return check

    def following(self, sysids: list[int]) -> Check:
        return all_of(
            *(self.mode(s, apm.COPTER_MODE_FOLLOW) for s in sysids),
            *(self.in_slot(s) for s in sysids),
        )

    def distance_m(self, sysid: int, since: tuple[float, float]) -> float:
        v = self.vehicle(sysid)
        n, e = LocalFrame(*since).to_ne(v.lat_deg, v.lon_deg)
        return math.hypot(n, e)

    def to(self, sysid: int, point: tuple[float, float]) -> tuple[float, float]:
        """North and east from copter `sysid` to `point`, metres."""
        v = self.vehicle(sysid)
        return LocalFrame(v.lat_deg, v.lon_deg).to_ne(*point)

    def near(self, sysid: int, point: tuple[float, float], radius_m: float, name: str) -> Check:
        def check() -> str:
            d = math.hypot(*self.to(sysid, point))
            return (
                ""
                if d <= radius_m
                else f"copter {sysid} within {radius_m:g} m of {name} ({d:.0f} m)"
            )

        return check

    def steer_to(self, sysid: int, point: tuple[float, float]) -> Callable[[], None]:
        """Fly copter `sysid` in LOITER on the sticks towards `point`, at the height it is: yaw
        onto it, then forward, full stick far off and less as it gets close."""

        def steer() -> None:
            v = self.vehicle(sysid)
            n, e = self.to(sysid, point)
            distance = math.hypot(n, e)
            error = (math.atan2(e, n) - v.heading_rad + math.pi) % (2 * math.pi) - math.pi
            yaw = 1500 + round(max(-200.0, min(200.0, math.degrees(error) * 8)))
            forward = round(min(500.0, distance * 25.0)) if abs(error) < math.radians(20) else 0
            if distance < 3.0:
                yaw = 1500  # the bearing swings about close in
            self.radio.sticks(pitch=1500 - forward, yaw=yaw, throttle=1500)

        return steer

    def where(self, sysid: int) -> tuple[float, float]:
        v = self.vehicle(sysid)
        return v.lat_deg, v.lon_deg

    # -- what the COP shows ------------------------------------------------------------

    def cop_formation(self, leader: int, followers: list[int]) -> Check:
        """The 3D viewer shows the leader, each follower with its slot, and point B."""

        def check() -> str:
            if self._cop_events is None:
                return ""
            frame = self.cop
            if frame is None:
                return "a frame in the COP"
            if frame["leader"] != leader:
                return f"COP leader {leader} (shows {frame['leader']})"
            shown = {v["sysid"]: v for v in frame["vehicles"]}
            for s in followers:
                v = shown.get(s)
                if v is None:
                    return f"copter {s} in the COP"
                if v["role"] != "follower" or v["slot"] is None:
                    return f"copter {s} a follower with its slot in the COP (is {v['role']})"
            if frame["target"] is None:
                return "point B in the COP"
            return ""

        return check

    def cop_modes(self, sysids: list[int]) -> Check:
        """The 3D viewer shows each copter's mode and arming as ArduPilot reports them."""

        def check() -> str:
            if self._cop_events is None:
                return ""
            if self.cop is None:
                return "a frame in the COP"
            shown = {v["sysid"]: v for v in self.cop["vehicles"]}
            for s in sysids:
                v, cop = self.vehicle(s), shown.get(s)
                if cop is None:
                    return f"copter {s} in the COP"
                if cop["mode"] != mode_name(v.mode) or cop["armed"] != v.armed:
                    return f"copter {s} in the COP as {mode_name(v.mode)} (shows {cop['mode']})"
            return ""

        return check


def rehearse(
    console: Console,
    panel: ScriptedPanel,
    radio: ScriptedRadio,
    viewer: "ViewServer | None",
    count: int,
    field: "SimField",
    land_radius_m: float,
) -> list[Result]:
    """Fly the mission. Returns every step's result; the last one failed if any did. Every
    drone must end within `land_radius_m` of B, the landing ring the viewer draws."""
    r = Rehearsal(console, panel, radio, viewer)
    try:
        _mission(r, count, field, land_radius_m)
    except StepError:
        pass
    return r.results


def _mission(r: Rehearsal, count: int, field: "SimField", land_radius_m: float) -> None:
    launcher, panel, radio = r.launcher, r.panel, r.radio
    lead = launcher.leader
    followers = [f.sysid for f in launcher.followers.values()]
    switch = {key: c for c, key in FOLLOWER_SWITCHES.items()}
    selector = {key: c for c, key in TAKEOVER_SWITCHES.items() if key in KEYS}
    first, last = launcher.followers[KEYS[0]], list(launcher.followers.values())[-1]
    G, L, F = apm.COPTER_MODE_GUIDED, apm.COPTER_MODE_LOITER, apm.COPTER_MODE_FOLLOW  # noqa: N806

    with r.step(f"{count} copters up, each controller linked: EKF and pre-arm checks pass"):
        r.wait(r.ready(count), 180)

    with r.step("power-up, gear DOWN: panel locked, nose LED red"):
        panel.gear_down, panel.rotary = True, MAG_OFF
        r.wait(all_of(r.live(False), r.nose(Led.RED), r.radio_on(None)), 5)

    with r.step("gear UP: panel live, nose LED amber (radio flies nothing)"):
        panel.gear_down = False
        r.wait(all_of(r.live(True), r.nose(Led.AMBER)), 5)

    with r.step("ALT ON: leader arms in GUIDED, holding on the ground"):
        panel.on.add(ALT)
        r.wait(all_of(r.armed(lead), r.mode(lead, G), r.on_ground(lead)), 30)

    with r.step("BEACON ON: radio flies the leader in LOITER, nose LED green"):
        panel.on.add(BEACON)
        r.wait(all_of(r.radio_on(lead), r.mode(lead, L), r.armed(lead), r.nose(Led.GREEN)), 15)

    with r.step("throttle up: leader climbs in LOITER, holds 15 m at mid stick"):
        radio.sticks(throttle=CLIMB_PWM)
        r.wait(r.above(lead, 15.0), 30)
        radio.sticks(throttle=1500)
        r.keep(all_of(r.above(lead, 13.0), r.mode(lead, L)), 3)

    def waiting_armed(sysids: list[int]) -> Check:
        return all_of(
            *(
                all_of(r.armed(s), r.mode(s, G), r.on_ground(s), r.stage(s, Stage.ARMING))
                for s in sysids
            )
        )

    with r.step(f"copter {last.sysid}'s controller off: its RC pre-arm check fails"):
        radio.power(last.sysid, on=False)
        r.wait(all_of(r.rc(last.sysid, False), r.prearm(last.sysid, False)), 15)

    names = ", ".join(SWITCH_NAMES[switch[f.key]] for f in launcher.followers.values())
    linked = [s for s in followers if s != last.sysid]
    with r.step(
        f"{names} ON: followers arm in GUIDED and wait for a phase; "
        f"copter {last.sysid} does not arm without its controller"
    ):
        panel.on.update(switch[f.key] for f in launcher.followers.values())
        r.wait(waiting_armed(linked), 30)
        r.keep(all_of(r.disarmed(last.sysid), r.stage(last.sysid, Stage.ARMING)), 5)

    with r.step(f"copter {last.sysid}'s controller on: it arms"):
        radio.power(last.sysid, on=True)
        r.wait(all_of(r.rc(last.sysid, True), waiting_armed([last.sysid])), 30)

    with r.step("rotary START: followers lift off to the leader's height plus a step, hold"):
        panel.rotary = MAG_START
        r.wait(all_of(*(all_of(r.stage(s, Stage.HOLDING), r.mode(s, G)) for s in followers)), 60)
        r.note(", ".join(f"{s}: {r.vehicle(s).rel_alt_m:.0f} m" for s in followers))

    with r.step("rotary BOTH: followers fly the square in FOLLOW; the COP shows the formation"):
        panel.rotary = MAG_BOTH
        r.wait(all_of(r.following(followers), r.cop_formation(lead, followers)), 90)

    with r.step("leader flies forward 15 s: the square keeps up"):
        start, worst = r.where(lead), 0.0

        def track() -> None:
            nonlocal worst
            worst = max(worst, *(r.slot_error_m(s) or 0.0 for s in followers))

        radio.sticks(pitch=FORWARD_PWM)
        r.keep(all_of(*(r.mode(s, F) for s in followers)), 15, during=track)
        radio.sticks()
        flown = r.distance_m(lead, start)
        if flown < 30.0:
            raise StepError(f"the leader to fly 30 m (flew {flown:.0f} m)")
        r.wait(r.following(followers), 60)
        r.note(f"leader flew {flown:.0f} m; worst slot error in flight {worst:.1f} m")

    with r.step("leader yaws right 90 degrees: the square stays put, north-up"):
        heading = r.vehicle(lead).heading_rad
        before = {s: r.where(s) for s in followers}

        def turned() -> str:
            delta = (r.vehicle(lead).heading_rad - heading) % (2 * math.pi)
            return (
                ""
                if 0.5 * math.pi <= delta < math.pi
                else f"90 deg yaw ({math.degrees(delta):.0f})"
            )

        radio.sticks(yaw=1700)
        r.wait(turned, 20)
        radio.sticks()
        r.wait(r.following(followers), 60)
        moved = max(r.distance_m(s, before[s]) for s in followers)
        if moved > 3.0:
            raise StepError(f"followers to stay within 3 m through the yaw (moved {moved:.1f} m)")
        r.note(f"followers moved {moved:.1f} m")

    with r.step("rotary START: pause; followers hold while the leader flies off"):
        panel.rotary = MAG_START
        r.wait(all_of(*(all_of(r.mode(s, G), r.stage(s, Stage.HOLDING)) for s in followers)), 10)
        r.run(4)  # brake to a stop
        held = {s: r.where(s) for s in followers}
        start = r.where(lead)
        radio.sticks(pitch=FORWARD_PWM)
        r.run(10)
        radio.sticks()
        drift = max(r.distance_m(s, held[s]) for s in followers)
        if drift > 3.0:
            raise StepError(f"followers to hold within 3 m (drifted {drift:.1f} m)")
        r.note(f"leader flew {r.distance_m(lead, start):.0f} m; followers drifted {drift:.1f} m")

    target = first.sysid
    avoid = apm.COPTER_MODE_AVOID_ADSB
    with r.step(
        f"the pilot climbs to copter {target}'s height and flies at it: it escapes and holds; "
        "the leader, on the radio, is never taken over"
    ):
        r.wait(
            all_of(
                r.param(lead, "AVD_F_ACTION", AVOID_REPORT),
                *(r.param(s, "AVD_F_ACTION", AVOID_ESCAPE) for s in followers),
            ),
            15,
        )
        closest = math.inf

        def steer() -> None:
            """Take the leader to copter `target`'s height, yaw onto it and fly at it."""
            nonlocal closest
            v, t = r.vehicle(lead), r.vehicle(target)
            n, e = LocalFrame(v.lat_deg, v.lon_deg).to_ne(t.lat_deg, t.lon_deg)
            error = (math.atan2(e, n) - v.heading_rad + math.pi) % (2 * math.pi) - math.pi
            yaw = 1500 + round(max(-200.0, min(200.0, math.degrees(error) * 8)))
            climb = t.rel_alt_m - v.rel_alt_m
            level = abs(climb) < 0.6
            # Clear of LOITER's mid-stick dead zone (THR_DZ), where it holds height.
            throttle = 1500 if level else 1500 + (CLIMB_STEP_PWM if climb > 0 else -CLIMB_STEP_PWM)
            pitch = FORWARD_PWM if abs(error) < math.radians(15) and level else 1500
            radio.sticks(pitch=pitch, yaw=yaw, throttle=throttle)
            closest = min(closest, r.separation_m(lead, target))

        r.wait(all_of(r.mode(target, avoid), r.mode(lead, L)), 60, during=steer)
        r.run(2, during=steer)
        radio.sticks(throttle=1500)
        r.wait(r.mode(target, G), 20)
        r.keep(all_of(r.mode(lead, L), r.mode(target, G)), 2)
        if closest < 2.0:
            raise StepError(f"copter {target} to stay 2 m clear of the leader ({closest:.1f} m)")
        r.note(f"closest {closest:.1f} m")

    with r.step("rotary BOTH: followers rejoin the leader"):
        panel.rotary = MAG_BOTH
        r.wait(r.following(followers), 90)

    lost = followers[-2] if len(followers) > 1 else followers[0]
    with r.step(f"copter {lost}'s controller dies in flight: radio failsafe, it keeps following"):
        radio.power(lost, on=False)
        r.wait(r.rc(lost, False), 10)
        r.keep(all_of(r.mode(lost, F), r.in_air(lost)), 5)
        radio.power(lost, on=True)
        r.wait(all_of(r.rc(lost, True), r.following(followers)), 30)

    nav = selector[first.key]
    with r.step(
        f"BEACON OFF, {SWITCH_NAMES[nav]} ON: radio flies copter {first.sysid}; "
        "the leader holds in GUIDED on its own idle controller"
    ):
        panel.on.discard(BEACON)
        r.wait(all_of(r.radio_on(None), r.mode(lead, G), r.nose(Led.AMBER)), 10)
        panel.on.add(nav)
        r.wait(all_of(r.radio_on(first.sysid), r.mode(first.sysid, L)), 10)
        others = [s for s in followers if s != first.sysid]
        r.keep(
            all_of(
                r.mode(lead, G),
                r.in_air(lead),
                r.mode(first.sysid, L),
                *(r.mode(s, F) for s in others),
            ),
            RELEASE_HOLD_S + 3,
        )
        if not r.vehicle(lead).rc_ok:
            raise StepError(f"copter {lead} to keep its RC link once the radio left it")

    with r.step(f"copter {first.sysid} on the sticks: LOITER, STABILIZE, LOITER again"):
        start = r.where(first.sysid)
        radio.sticks(roll=1700)
        r.run(5)
        radio.sticks()
        moved = r.distance_m(first.sysid, start)
        if moved < 8.0:
            raise StepError(f"copter {first.sysid} to fly 8 m on the sticks (flew {moved:.0f} m)")
        radio.mode_switch(apm.COPTER_MODE_STABILIZE)
        r.wait(r.mode(first.sysid, apm.COPTER_MODE_STABILIZE), 5)
        r.keep(r.above(first.sysid, 5.0), 3)
        radio.mode_switch(L)
        r.wait(r.mode(first.sysid, L), 5)
        r.note(f"flew {moved:.0f} m sideways")

    with r.step(
        f"{SWITCH_NAMES[nav]} OFF, BEACON ON: radio back on the leader; "
        f"copter {first.sysid} rejoins in FOLLOW"
    ):
        panel.on.discard(nav)
        r.wait(all_of(r.radio_on(None), r.mode(first.sysid, F)), 10)
        panel.on.add(BEACON)
        r.wait(all_of(r.radio_on(lead), r.mode(lead, L), r.following(followers)), 90)

    pitot = switch[last.key]
    with r.step(f"{SWITCH_NAMES[pitot]} OFF: copter {last.sysid} lands; ON again: it rejoins"):
        panel.on.discard(pitot)
        r.wait(
            all_of(r.mode(last.sysid, apm.COPTER_MODE_LAND), r.stage(last.sysid, Stage.LANDING)), 10
        )
        r.run(3)
        panel.on.add(pitot)
        r.wait(all_of(r.mode(last.sysid, F), r.in_air(last.sysid)), 15)
        r.wait(r.following(followers), 60)

    with r.step(
        "the pilot's controller dies flying the leader: it brakes, the square holds on it; "
        "link back, a flick of the mode switch takes it again"
    ):
        radio.power(lead, on=False)
        r.wait(all_of(r.rc(lead, False), r.mode(lead, apm.COPTER_MODE_BRAKE)), 10)
        r.keep(all_of(r.in_air(lead), *(r.mode(s, F) for s in followers)), 3)
        radio.power(lead, on=True)
        r.wait(r.rc(lead, True), 10)
        radio.mode_switch(apm.COPTER_MODE_STABILIZE)
        r.wait(r.mode(lead, apm.COPTER_MODE_STABILIZE), 5)
        radio.mode_switch(L)
        r.wait(all_of(r.mode(lead, L), r.following(followers)), 60)

    rtl = apm.COPTER_MODE_RTL
    with r.step("rotary R: followers return home; the leader stays on the radio"):
        panel.rotary = MAG_R
        r.wait(all_of(*(r.mode(s, rtl) for s in followers), r.mode(lead, L)), 10)
        r.keep(all_of(r.mode(lead, L), r.radio_on(lead)), 3)

    with r.step("BEACON OFF under R: the leader let go returns home too"):
        panel.on.discard(BEACON)
        r.wait(all_of(r.radio_on(None), r.mode(lead, rtl), r.stage(lead, Stage.RETURNING)), 10)
        r.keep(all_of(r.mode(lead, rtl), *(r.mode(s, rtl) for s in followers)), 3)

    with r.step("rotary BOTH: return home called off; the leader holds, followers rejoin it"):
        panel.rotary = MAG_BOTH
        r.wait(
            all_of(
                r.mode(lead, G),
                r.stage(lead, Stage.HOLDING),
                r.following(followers),
                r.cop_formation(lead, followers),
            ),
            120,
        )

    with r.step("COWL ON: auto select puts the radio on the leader"):
        panel.on.add(COWL)
        r.wait(all_of(r.radio_on(lead), r.mode(lead, L)), 10)

    with r.step("rotary OFF: followers start landing; BOTH before touchdown calls it off"):
        panel.rotary = MAG_OFF
        r.wait(all_of(*(r.mode(s, apm.COPTER_MODE_LAND) for s in followers), r.mode(lead, L)), 10)
        r.run(2)
        panel.rotary = MAG_BOTH
        r.wait(r.following(followers), 90)

    # The mission proper: the square to A, hold there, then each drone on to B and down.
    a, b = field.point_a, field.point_b
    with r.step(f"BOTH: the pilot flies the swarm to point A ({math.hypot(*r.to(lead, a)):.0f} m)"):
        r.wait(r.near(lead, a, 5.0, "A"), 180, during=r.steer_to(lead, a))
        radio.sticks(throttle=1500)
        r.wait(all_of(r.following(followers), r.cop_formation(lead, followers)), 60)

    with r.step("rotary L at A: the swarm holds there in GUIDED"):
        panel.rotary = MAG_L
        r.wait(all_of(*(all_of(r.mode(s, G), r.stage(s, Stage.HOLDING)) for s in followers)), 10)
        r.keep(r.near(lead, a, 8.0, "A"), 3)

    order = [lead, *followers]
    for k, s in enumerate(order):
        # Each drone has its own spot on a ring round B, so none lands on another.
        angle = 2.0 * math.pi * k / len(order)
        spot = LocalFrame(*b).to_latlon(B_RING_M * math.cos(angle), B_RING_M * math.sin(angle))
        with r.step(f"to B: auto select gives the radio copter {s}; flown to B, landed, disarmed"):
            r.wait(all_of(r.radio_on(s), r.mode(s, L), r.nose(Led.GREEN)), 20)
            r.wait(r.near(s, spot, 2.0, "its spot by B"), 240, during=r.steer_to(s, spot))
            radio.sticks(throttle=PWM_MIN)
            r.wait(r.on_ground(s), 60)
            radio.sticks(throttle=PWM_MIN, yaw=PWM_MIN)
            r.wait(r.disarmed(s), 15)
            radio.sticks(throttle=1500)  # the next drone gets the radio at once
            r.note(f"down {math.hypot(*r.to(s, b)):.1f} m from B")

    with r.step("every drone down by B; gear DOWN locks the panel; the COP agrees"):
        r.wait(all_of(*(r.near(s, b, land_radius_m, "B") for s in order)), 5)
        panel.gear_down = True
        r.wait(all_of(r.radio_on(None), r.live(False), r.nose(Led.RED), r.cop_modes(order)), 10)

    with r.step("panel reset, gear UP, BEACON ON: throttle down, yaw right arms the leader"):
        panel.on, panel.rotary = set(), MAG_OFF
        r.run(1)
        panel.gear_down = False
        panel.on.add(BEACON)
        radio.sticks(throttle=PWM_MIN)
        r.wait(all_of(r.live(True), r.radio_on(lead), r.mode(lead, L)), 10)
        radio.sticks(throttle=PWM_MIN, yaw=2000)
        r.wait(r.armed(lead), 10)
        radio.sticks(throttle=PWM_MIN)

    with r.step("hop to 5 m; gear DOWN: the radio lets go, it lands, panel locked"):
        radio.sticks(throttle=CLIMB_PWM)
        r.wait(r.above(lead, 5.0), 20)
        radio.sticks(throttle=1500)
        panel.gear_down = True
        r.wait(all_of(r.radio_on(None), r.live(False), r.mode(lead, apm.COPTER_MODE_LAND)), 5)
        r.wait(r.disarmed(lead), 60)


def summary(results: list[Result]) -> list[str]:
    width = max((len(r.name) for r in results), default=0)
    lines = [
        f"{'PASS' if r.ok else 'FAIL'}  {r.name:<{width}}  {r.seconds:5.1f} s"
        + (f"  {r.note}" if r.note else "")
        for r in results
    ]
    passed = sum(r.ok for r in results)
    failed = len(results) - passed
    lines.append(
        f"{passed} passed" + (f", 1 failed: {results[-1].name}" if failed else ", mission complete")
    )
    return lines
