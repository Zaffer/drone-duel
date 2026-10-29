"""The session: one vehicle in one slot, spawned at a scenario's start pad (DESIGN.md §6.4).

Commands (start, switch, reset, destroy, throw, stop) run one at a time on the session's own
thread, in order. A new start or switch cancels a spawn still in progress. The pilot's sticks,
the ground and the wind go straight through, from any thread.
"""

import logging
import queue
import threading
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Any

from drone_duels import ports
from drone_duels.content.schema import Vehicle, Wind
from drone_duels.content.store import Content
from drone_duels.server.protocol import SessionState, VideoSystem
from drone_duels.session.base import (
    LOGS,
    Cancelled,
    Events,
    Firmware,
    FirmwareError,
    NoFirmware,
    PhysicsProcess,
    SpawnContext,
)
from drone_duels.world.elevation import ElevationError, ground_elevation_m

log = logging.getLogger(__name__)

RC_TIMEOUT_S = 0.2
"""No sticks from the page for this long: the radio link is lost (DESIGN.md §8)."""
PHYSICS_READY_S = 30.0


def make_firmware(ctx: SpawnContext, events: Events, physics: PhysicsProcess) -> Firmware:
    kind = ctx.vehicle.firmware.kind
    if kind == "ardupilot-copter":
        from drone_duels.firmware.ardupilot import ArduCopter

        return ArduCopter(ctx, events, physics)
    if kind == "inav":
        from drone_duels.firmware.inav import Inav

        return Inav(ctx, events, physics)
    return NoFirmware(ctx, events, physics)


@dataclass
class _Command:
    action: str
    scenario: str | None = None
    vehicle: str | None = None
    pad: int = 0
    reason: str = ""
    of: object = None


class Session:
    def __init__(
        self,
        content: Content,
        events: Events,
        slot: int = 0,
        firmware_factory: Callable[[SpawnContext, Events, PhysicsProcess], Firmware] = (
            make_firmware
        ),
    ) -> None:
        self.content = content
        self.events = events
        self.ports = ports.slot(slot)
        self.firmware_factory = firmware_factory
        self.state: SessionState = "idle"
        self.step = ""
        self.scenario: str | None = None
        self.vehicle: str | None = None
        self.pad = 0
        self.video: VideoSystem = "analog"
        self.wind: Wind | None = None
        self.dynamics_on = False
        self.origin: tuple[float, float, float] | None = None
        self.heading_deg = 0.0
        self._firmware: Firmware | None = None
        self._physics: PhysicsProcess | None = None
        self._cancel = threading.Event()
        self._commands: queue.Queue[_Command | None] = queue.Queue()
        self._lock = threading.Lock()
        self._last_rc = 0.0
        self._rc_live = False
        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._run, name="session", daemon=True)
        self._watchdog = threading.Thread(target=self._watch, name="session-watch", daemon=True)

    # Lifecycle.

    def start(self) -> None:
        self._worker.start()
        self._watchdog.start()

    def close(self) -> None:
        self._cancel.set()
        self._commands.put(_Command("stop"))
        self._commands.put(None)
        self._worker.join(15)
        self._stop.set()
        self._watchdog.join(2)

    # Commands, from the page.

    def command(
        self,
        action: str,
        scenario: str | None = None,
        vehicle: str | None = None,
        pad: int = 0,
    ) -> None:
        if action in ("start", "switch", "stop"):
            self._cancel.set()  # overtake a spawn in progress
        self._commands.put(_Command(action, scenario, vehicle, pad))

    def send_rc(self, us: Sequence[int]) -> None:
        self._last_rc = time.monotonic()
        self._rc_live = True
        firmware = self._firmware
        if firmware is not None and self.state == "ready":
            firmware.send_rc(us)

    def set_ground(self, lat: float, lon: float, alt: float) -> None:
        if self._physics:
            self._physics.send({"t": "ground", "lat": lat, "lon": lon, "alt": alt})

    def set_wind(self, wind: Wind, gust_now: bool = False) -> None:
        self.wind = wind
        if self._physics:
            self._physics.send({"t": "wind", **wind.model_dump(), "gust_now": gust_now})

    def set_video(self, system: VideoSystem) -> None:
        """The OSD grid follows the video system, and the firmware takes it only at boot, so a
        flying vehicle respawns at its start (DESIGN.md §10)."""
        if system == self.video:
            return
        self.video = system
        if self.vehicle and self.state in ("ready", "destroyed", "failed"):
            self.command("switch", vehicle=self.vehicle)
        else:
            self._status()

    def set_dynamics(self, on: bool) -> None:
        self.dynamics_on = on
        if self._physics:
            self._physics.send({"t": "dynamics", "on": on})

    def status(self) -> dict[str, Any]:
        return {
            "ch": "status",
            "state": self.state,
            "scenario": self.scenario,
            "vehicle": self.vehicle,
            "pad": self.pad,
            "step": self.step,
            "video": self.video,
            "origin": self.origin,
            "heading_deg": self.heading_deg,
        }

    # The worker.

    def _run(self) -> None:
        while True:
            cmd = self._commands.get()
            if cmd is None:
                return
            try:
                self._do(cmd)
            except Exception as exc:  # a bug: say so on the page, keep the session alive
                log.exception("session %s", cmd.action)
                self._fail(f"{cmd.action}: {exc}")

    def _do(self, cmd: _Command) -> None:
        if cmd.action == "start":
            scenario = self.content.scenario(cmd.scenario or "")
            vehicle = cmd.vehicle or scenario.default_vehicle
            if vehicle not in scenario.vehicles:
                raise ValueError(f"{scenario.id} does not offer {vehicle}")
            self.scenario, self.pad = scenario.id, cmd.pad
            self.wind = self.content.locations[scenario.location].wind
            self._spawn(vehicle)
        elif cmd.action == "switch":
            if self.scenario is None:
                raise ValueError("no scenario started")
            vehicle = cmd.vehicle or self.vehicle
            scenario = self.content.scenario(self.scenario)
            if vehicle not in scenario.vehicles:
                raise ValueError(f"{scenario.id} does not offer {vehicle}")
            self._spawn(vehicle)
        elif cmd.action == "reset":
            if self.vehicle is not None:
                self._spawn(self.vehicle)
        elif cmd.action == "destroy":
            self._teardown()
            self.state, self.step = "destroyed", "destroyed"
            self._status()
        elif cmd.action == "throw":
            if self._physics:
                self._physics.send({"t": "throw"})
        elif cmd.action == "lost":
            if cmd.of is self._firmware and self.state == "ready":
                self._fail(cmd.reason)
        elif cmd.action == "stop":
            self._teardown()
            self.state, self.step, self.scenario, self.vehicle = "idle", "", None, None
            self.origin = None
            self._status()

    def _spawn(self, vehicle_id: str) -> None:
        self._teardown()
        self._cancel = threading.Event()
        cancel = self._cancel
        assert self.scenario is not None
        scenario = self.content.scenario(self.scenario)
        location = self.content.locations[scenario.location]
        vehicle: Vehicle = self.content.vehicle(vehicle_id)
        pad = location.pads[min(self.pad, len(location.pads) - 1)]
        self.vehicle = vehicle_id
        self.state = "starting"
        self._set_step("finding the ground")
        ground = location.ground_alt_m
        if ground == "auto":
            try:
                ground = ground_elevation_m(location.origin[0], location.origin[1])
            except ElevationError as exc:
                self._fail(f"ground height of {location.name}: {exc}")
                return
        self.origin = (pad.at[0], pad.at[1], float(ground))
        self.heading_deg = pad.heading_deg
        ctx = SpawnContext(
            vehicle=vehicle,
            location=location,
            pad=pad,
            ground_alt_m=float(ground),
            ports=self.ports,
            video=self.video,
            wind=self.wind or location.wind,
            workdir=LOGS / "slots" / str(self.ports.slot) / vehicle.id,
            cancel=cancel,
        )
        physics = PhysicsProcess(ctx, self.events)
        firmware = self.firmware_factory(ctx, self.events, physics)
        self._physics, self._firmware = (
            physics if vehicle.physics.kind != "browser-truck" else None
        ), firmware
        try:
            if self._physics is not None:
                self._set_step("starting the physics")
                physics.start()
                if not physics.wait_status(lambda s: s.get("listening", False), PHYSICS_READY_S):
                    raise FirmwareError(
                        f"the physics did not start: see {ctx.workdir / 'physics.log'}"
                    )
                if self.dynamics_on:
                    physics.send({"t": "dynamics", "on": True})
            ctx.check()
            self._set_step(f"starting {firmware.name}")
            firmware.start()
            firmware.wait_ready(self._set_step)
            ctx.check()
        except Cancelled:
            return  # the next command tears it down
        except FirmwareError as exc:
            self._fail(str(exc))
            return
        self.state, self.step = "ready", ""
        self._status()
        self.events.publish(
            {"ch": "log", "level": "info", "source": "session", "text": f"{vehicle.name} ready"}
        )

    def _teardown(self) -> None:
        firmware, physics = self._firmware, self._physics
        self._firmware = self._physics = None
        if firmware is not None:
            firmware.stop()
        if physics is not None:
            physics.stop()

    def _set_step(self, step: str) -> None:
        self.step = step
        self._status()

    def _fail(self, why: str) -> None:
        self._teardown()
        self.state, self.step = "failed", why
        self._status()
        self.events.publish({"ch": "log", "level": "error", "source": "session", "text": why})

    def _status(self) -> None:
        self.events.publish(self.status())

    # The watchdog: radio link loss, and firmware or physics dying under us.

    def _watch(self) -> None:
        while not self._stop.wait(0.05):
            firmware = self._firmware
            if self._rc_live and time.monotonic() - self._last_rc > RC_TIMEOUT_S:
                self._rc_live = False
                if firmware is not None:
                    firmware.rc_lost()
            if self.state != "ready":
                continue
            physics = self._physics
            why = ""
            if firmware is not None and not firmware.alive():
                why = f"{firmware.name} stopped: see its log under {LOGS / 'slots'}"
            elif physics is not None and not physics.alive():
                why = f"the physics stopped: see physics.log under {LOGS / 'slots'}"
            if why:
                self._commands.put(_Command("lost", reason=why, of=firmware))
                time.sleep(0.5)
