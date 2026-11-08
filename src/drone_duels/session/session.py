"""The session: one vehicle in one slot, spawned at a location's start pad (DESIGN.md §6.4).

Commands (start, switch, reset, destroy, throw, target, stop) run one at a time on the session's
own thread, in order. A new start or switch cancels a spawn still in progress. The pilot's
sticks, the ground, the wind and hits go straight through, from any thread.

A target (Shoot Down) is the firmware flying itself, as a ground station would fly it, or (for
Betaflight, which takes no such commands) as a pilot would on its own modes (Firmware.hover,
land): queued as a command, so a `reset` then `target hover` from the page respawns the vehicle
first and then takes off. A hit goes straight to the physics
(physics/quad/model.py), and the firmware flies on with what it leaves.
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
"""No sticks for this long (the server's radio or the page's): the radio link is lost (DESIGN.md
§8)."""
LAUNCH_HIGH_US = 1665
"""A launch channel above this is high (the switch's top third, as mapping.switch_pos)."""
PHYSICS_READY_S = 30.0


def make_firmware(ctx: SpawnContext, events: Events, physics: PhysicsProcess) -> Firmware:
    kind = ctx.vehicle.firmware.kind
    if kind == "ardupilot-copter":
        from drone_duels.firmware.ardupilot import ArduCopter

        return ArduCopter(ctx, events, physics)
    if kind == "inav":
        from drone_duels.firmware.inav import Inav

        return Inav(ctx, events, physics)
    if kind == "betaflight":
        from drone_duels.firmware.betaflight import Betaflight

        return Betaflight(ctx, events, physics)
    return NoFirmware(ctx, events, physics)


@dataclass
class _Command:
    action: str
    location: str | None = None
    vehicle: str | None = None
    pad: int = 0
    reason: str = ""
    of: object = None
    height_m: float = 0.0
    payload: str | None = None


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
        self.location: str | None = None
        self.vehicle: str | None = None
        self.payload: str | None = None
        """The payload fitted to it (content/payloads), kept by a switch."""
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
        self._spec: Vehicle | None = None
        self._launch_high = True
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
        location: str | None = None,
        vehicle: str | None = None,
        pad: int = 0,
        payload: str | None = None,
    ) -> None:
        if action in ("start", "switch", "stop"):
            self._cancel.set()  # overtake a spawn in progress
        self._commands.put(_Command(action, location, vehicle, pad, payload=payload))

    def send_rc(self, us: Sequence[int]) -> None:
        """The pilot's channels, µs, from the server's radio (rc.pilot) or the page's."""
        self._last_rc = time.monotonic()
        self._rc_live = True
        firmware = self._firmware
        if firmware is not None and self.state == "ready":
            firmware.send_rc(us)
            self._launch(us)

    def _launch(self, us: Sequence[int]) -> None:
        """A hand-launched wing is thrown when its launch switch flicks high, whoever flies it
        (DESIGN.md §12); a switch already high at the start waits to go low first."""
        aux = self._spec.aux if self._spec else None
        if aux is None or aux.launch is None or len(us) < aux.launch:
            return
        high = us[aux.launch - 1] > LAUNCH_HIGH_US
        if high and not self._launch_high:
            self.command("throw")
        self._launch_high = high

    def target(self, action: str, height_m: float = 0.0) -> None:
        """Fly the vehicle as a target: "hover" `height_m` above its pad, or "land" (in order,
        after any command before it: a reset first respawns it)."""
        self._commands.put(_Command("target", reason=action, height_m=height_m))

    def go(
        self,
        point: tuple[float, float, float],
        vel_ned: tuple[float, float, float] | None,
        yaw_deg: float | None,
    ) -> None:
        """Fly the target on (protocol.TargetMsg "go"): straight to the firmware, a few times a
        second, while it is ready and has not crashed."""
        firmware, physics = self._firmware, self._physics
        if self.state != "ready" or firmware is None:
            return
        if physics is not None and physics.status.get("phase") == "crashed":
            return
        firmware.go(point, vel_ned, yaw_deg)

    def path(self, points: Sequence[tuple[float, float]], height_m: float) -> None:
        """Fly the target round a path (protocol.TargetMsg "path"): straight to the firmware,
        while it is ready and has not crashed."""
        firmware, physics = self._firmware, self._physics
        if self.state != "ready" or firmware is None:
            return
        if physics is not None and physics.status.get("phase") == "crashed":
            return
        firmware.path(points, height_m)

    def hit(
        self,
        part: str,
        motor: int | None,
        impulse_ned: tuple[float, float, float],
        at_frd: tuple[float, float, float],
    ) -> None:
        """A bullet struck the vehicle (protocol.HitMsg): straight to the physics."""
        if self._physics:
            self._physics.send(
                {"t": "hit", "part": part, "motor": motor, "impulse_ned": list(impulse_ned),
                 "at_frd": list(at_frd)}
            )

    def set_ground(self, lat: float, lon: float, alt: float) -> None:
        if self._physics:
            self._physics.send({"t": "ground", "lat": lat, "lon": lon, "alt": alt})

    def set_wind(self, wind: Wind, gust_now: bool = False) -> None:
        self.wind = wind
        if self._physics:
            self._physics.send({"t": "wind", "wind": wind.model_dump(), "gust_now": gust_now})

    def set_video(self, system: VideoSystem) -> None:
        """The OSD grid follows the video system: in flight where the firmware can (ArduPilot),
        else the vehicle respawns at its start, the firmware taking it at boot (DESIGN.md §10)."""
        if system == self.video:
            return
        self.video = system
        firmware = self._firmware
        if self.state == "ready" and firmware is not None and firmware.set_video(system):
            self._status()
        elif self.vehicle and self.state in ("ready", "destroyed", "failed"):
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
            "location": self.location,
            "vehicle": self.vehicle,
            "pad": self.pad,
            "step": self.step,
            "video": self.video,
            "origin": self.origin,
            "heading_deg": self.heading_deg,
            "payload": (
                self._spec.payload.model_dump() if self._spec and self._spec.payload else None
            ),
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
            location = self.content.location(cmd.location or "")
            vehicle = self.content.vehicle(cmd.vehicle or "").id
            if cmd.payload is not None:
                self.content.payload(cmd.payload)  # there is one
            self.location, self.pad, self.payload = location.id, cmd.pad, cmd.payload
            self.wind = location.wind
            self._spawn(vehicle)
        elif cmd.action == "switch":
            if self.location is None:
                raise ValueError("nothing started")
            self._spawn(self.content.vehicle(cmd.vehicle or self.vehicle or "").id)
        elif cmd.action == "reset":
            firmware, physics = self._firmware, self._physics
            if (
                self.state == "ready"
                and firmware is not None
                and firmware.resets_in_place
                and physics is not None
            ):
                firmware.reset_in_place()
                physics.send({"t": "reset"})
                # Still ready, but said again: a page waiting on the reset (a race back on its
                # grid) hears that it is done.
                self._status()
            elif self.vehicle is not None:
                self._spawn(self.vehicle)
        elif cmd.action == "destroy":
            self._teardown()
            self.state, self.step = "destroyed", "destroyed"
            self._status()
        elif cmd.action == "throw":
            if self._physics:
                self._physics.send({"t": "throw"})
        elif cmd.action == "target":
            self._target(cmd.reason, cmd.height_m)
        elif cmd.action == "lost":
            if cmd.of is self._firmware and self.state == "ready":
                self._fail(cmd.reason)
        elif cmd.action == "stop":
            self._teardown()
            self.state, self.step, self.location, self.vehicle = "idle", "", None, None
            self.payload = None
            self.origin = None
            self._status()

    def _target(self, action: str, height_m: float) -> None:
        firmware, physics = self._firmware, self._physics
        why = ""
        if self.state != "ready" or firmware is None:
            why = "nothing ready to fly"
        elif physics is not None and physics.status.get("phase") == "crashed":
            why = "it has crashed: reset it first"
        if why or firmware is None:
            self.events.publish(
                {"ch": "log", "level": "warn", "source": "session", "text": f"{action}: {why}"}
            )
        elif action == "hover":
            firmware.hover(height_m)
        else:
            firmware.land()

    def _spawn(self, vehicle_id: str) -> None:
        self._teardown()
        self._cancel = threading.Event()
        cancel = self._cancel
        assert self.location is not None
        location = self.content.location(self.location)
        vehicle: Vehicle = self.content.fitted(vehicle_id, self.payload)
        pad = location.pads[min(self.pad, len(location.pads) - 1)]
        self.vehicle = vehicle_id
        self._spec = vehicle
        self._launch_high = True
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
            # Its own with a payload: ArduPilot keeps its parameters there between boots.
            workdir=LOGS / "slots" / str(self.ports.slot)
            / (vehicle.id + (f"+{self.payload}" if self.payload else "")),
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
