"""The physics process: one vehicle's flight model, playing the world to its firmware in lockstep
(or in real time, for Betaflight), and talking to the server over a UDP pair (DESIGN.md §3).

    python -m drone_duels.physics.runner --kind quad --airframe racer5in --slot 0 \\
        --origin LAT,LON,ALT --heading DEG --wind JSON --firmware ardupilot-copter

The backend (quad: ArduPilot's JSON physics, or Betaflight's simulator protocol; wing: X-Plane's
protocol for INAV) runs its own loop with the firmware on a thread. This loop, at LOOP_HZ, takes
the server's control datagrams (JSON, `t` names each: rc, ground, wind, reset, throw, hit,
dynamics, stop) and sends back the true pose (b"P" + protocol.POSE), the status (twice a second,
and on change: its phase, and what hits have taken) and, when asked, the dynamics.
"""

import argparse
import json
import logging
import socket
import struct
import sys
import time
from typing import Any, Protocol

from drone_duels import ports

log = logging.getLogger("physics")

LOOP_HZ = 240.0
POSE_HZ = 120.0
DYNAMICS_HZ = 30.0
STATUS_S = 0.5
RC_TIMEOUT_S = 1.0
"""No sticks from the server this long: the receiver's link is lost."""
POSE = struct.Struct("<18d")


class Backend(Protocol):
    """A flight model behind its firmware's simulator protocol."""

    def start(self) -> None:
        """Listen for the firmware, and run the lockstep loop on a thread of its own."""
        ...

    def stop(self) -> None: ...

    def connected(self) -> bool:
        """Whether the firmware is stepping the physics."""
        ...

    def phase(self) -> str:
        """What the vehicle is doing: "ground", "hand", "flying", "crashed"..."""
        ...

    def pose(self) -> tuple[float, ...] | None:
        """The true state, protocol.POSE_FIELDS in order; None before the first step."""
        ...

    def rc(self, us: list[int] | None) -> None:
        """The pilot's channels (µs, from channel 1), for a firmware that takes its RC through
        the simulator (INAV); None: the link is lost."""
        ...

    def ground(self, lat: float, lon: float, alt: float) -> None:
        """The ground's height above sea level at a point, from the map."""
        ...

    def wind(self, wind: dict[str, Any], gust_now: bool) -> None:
        """The wind now: content's Wind, as a dict; `gust_now` starts a gust at once."""
        ...

    def reset(self) -> None:
        """Back to the start, at rest."""
        ...

    def throw(self) -> None:
        """Throw a hand-launched aircraft (nothing for others)."""
        ...

    def hit(
        self,
        part: str,
        motor: int | None,
        impulse_ned: tuple[float, float, float],
        at_frd: tuple[float, float, float],
    ) -> None:
        """A bullet struck `part` (protocol.HitMsg), leaving momentum `impulse_ned` (N·s) at
        `at_frd` (m from the centre of mass, body FRD); `motor` ArduPilot's 1-4 for a rotor's
        part. A model without damage may ignore it."""
        ...

    def damage(self) -> dict[str, Any] | None:
        """What hits have taken (protocol.Damage), or None if nothing since the reset."""
        ...

    def dynamics(self) -> dict[str, Any] | None:
        """The flight model at work, for the dynamics overlay; None if it has none."""
        ...


def make_backend(
    kind: str,
    airframe: str,
    slot: int,
    origin: tuple[float, float, float],
    heading_deg: float,
    wind: dict[str, Any],
    firmware: str = "ardupilot-copter",
    payload: dict[str, Any] | None = None,
) -> Backend:
    if kind == "quad":
        from drone_duels.physics.quad.backend import QuadBackend

        return QuadBackend(airframe, ports.slot(slot), origin, heading_deg, wind, firmware,
                           payload)
    if kind == "wing":
        from drone_duels.physics.wing.backend import WingBackend

        return WingBackend(airframe, ports.slot(slot), origin, heading_deg, wind)
    raise SystemExit(f"no physics of kind {kind!r}")


def run(backend: Backend, slot: int) -> None:
    p = ports.slot(slot)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", p.physics_control))
    sock.setblocking(False)
    server = ("127.0.0.1", p.physics_state)

    def send(msg: dict[str, Any]) -> None:
        try:
            sock.sendto(json.dumps(msg, separators=(",", ":")).encode(), server)
        except OSError:
            pass

    backend.start()
    dynamics_on = False
    last_rc = 0.0
    rc_live = False
    next_pose = next_dyn = next_status = 0.0
    last_status: dict[str, Any] = {}
    period = 1.0 / LOOP_HZ
    try:
        while True:
            now = time.monotonic()
            while True:
                try:
                    data = sock.recv(65536)
                except BlockingIOError:
                    break
                try:
                    msg = json.loads(data)
                except ValueError:
                    continue
                kind = msg.get("t")
                if kind == "rc":
                    backend.rc(msg["us"])
                    last_rc, rc_live = now, True
                elif kind == "ground":
                    backend.ground(msg["lat"], msg["lon"], msg["alt"])
                elif kind == "wind":
                    backend.wind(msg["wind"], bool(msg.get("gust_now", False)))
                elif kind == "reset":
                    backend.reset()
                elif kind == "throw":
                    backend.throw()
                elif kind == "hit":
                    motor = msg.get("motor")
                    backend.hit(
                        str(msg["part"]),
                        int(motor) if motor is not None else None,
                        _vec3(msg["impulse_ned"]),
                        _vec3(msg["at_frd"]),
                    )
                elif kind == "dynamics":
                    dynamics_on = bool(msg.get("on"))
                elif kind == "stop":
                    return
            if rc_live and now - last_rc > RC_TIMEOUT_S:
                rc_live = False
                backend.rc(None)
            if now >= next_pose:
                next_pose = now + 1.0 / POSE_HZ
                pose = backend.pose()
                if pose is not None:
                    try:
                        sock.sendto(b"P" + POSE.pack(*pose), server)
                    except OSError:
                        pass
            if dynamics_on and now >= next_dyn:
                next_dyn = now + 1.0 / DYNAMICS_HZ
                dyn = backend.dynamics()
                if dyn is not None:
                    send({"t": "dynamics", **dyn})
            status = {"t": "status", "listening": True, "connected": backend.connected(),
                      "phase": backend.phase(), "damage": backend.damage()}
            if status != last_status or now >= next_status:
                next_status = now + STATUS_S
                last_status = status
                send(status)
            time.sleep(max(0.0, period - (time.monotonic() - now)))
    finally:
        backend.stop()
        sock.close()


def _vec3(v: Any) -> tuple[float, float, float]:  # noqa: ANN401 - JSON
    x, y, z = (float(c) for c in v)
    return (x, y, z)


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(prog="drone_duels.physics.runner")
    parser.add_argument("--kind", required=True, choices=["quad", "wing"])
    parser.add_argument("--airframe", required=True)
    parser.add_argument("--slot", type=int, default=0)
    parser.add_argument("--origin", required=True, help="LAT,LON,ALT (ground, m above sea level)")
    parser.add_argument("--heading", type=float, default=0.0)
    parser.add_argument("--wind", default="{}", help="the location's wind, as JSON")
    parser.add_argument("--firmware", default="ardupilot-copter",
                        help="the firmware flying it (content's FirmwareKind)")
    parser.add_argument("--payload", default=None,
                        help="what it carries (content's PayloadSpec), as JSON")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s",
                        stream=sys.stdout)
    lat, lon, alt = (float(x) for x in args.origin.split(","))
    payload = json.loads(args.payload) if args.payload else None
    backend = make_backend(args.kind, args.airframe, args.slot, (lat, lon, alt), args.heading,
                           json.loads(args.wind), args.firmware, payload)
    run(backend, args.slot)


if __name__ == "__main__":
    main()
