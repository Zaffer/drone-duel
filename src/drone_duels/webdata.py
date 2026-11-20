"""The web version's data (DESIGN.md §18): what the page asks the game's server for, written
out as static files, so a page with no server behind it (web/src/sim/, GitHub Pages) loads the
same things.

`write(out)` puts under `out`:

    content.json            protocol.ContentMsg: the content the web version flies. Vehicles: the
                            ones whose firmware and physics run in the page (WEB_FIRMWARE,
                            WEB_PHYSICS), so Betaflight's 5" quad and the truck. A location's
                            ground left "auto" is looked up now (world.elevation): the page has
                            nobody to ask.
    config.json             protocol.ConfigMsg, with no map keys: the page is public, so a key in
                            it would be anyone's. The player may give their own (web/src/net/).
    sim.json                `sim_data`: per vehicle, what its simulator in the page needs: the
                            firmware's CLI settings by video system, and the airframe as
                            SkyFlow-Dynamics packs it (the flat parameter vector of the JAX
                            backend the game's own physics runs, so the generated C in
                            web/wasm/sfd.c flies the same numbers).
    osd/font/betaflight.png Betaflight's OSD font, as the server serves it.

`duels web` writes it beside the page's web build (cli.py).

`generated()` is what is checked in for the web version and made here (`duels gen-web`;
tests/test_web.py and scripts/check.sh hold it current, as the protocol's schema is):

    web/test/fixtures/sim-data.json    `sim_data` and the content, for the page's tests of its
                                       simulator (web/test/sim-*.test.ts).
    web/test/fixtures/sim-flight.json  `reference_flight`: a flight of the game's own quad model
                                       (physics.quad.model, on the JAX backend), which the page's
                                       model (web/src/sim/quad.ts, on the generated C) must fly
                                       the same.
    web/wasm/sfd.c, sfd.h              SkyFlow-Dynamics' generated C backend, when the installed
                                       skyflow-dynamics has one (backends/c.py); until then they
                                       are kept as checked in, and tests/test_web.py holds them
                                       to the JAX backend by compiling and flying them.
"""

import json
from pathlib import Path
from typing import Any

from drone_duels import __version__
from drone_duels.content.schema import Location, Vehicle
from drone_duels.content.store import Content, load
from drone_duels.firmware.betaflight import commands
from drone_duels.firmware.inav import load_settings
from drone_duels.server.protocol import ConfigMsg, ContentMsg, MapKeys, VideoSystem
from drone_duels.session.base import settings_files
from drone_duels.video.osd import BETAFLIGHT_FONT, mcm_png
from drone_duels.world.elevation import ground_elevation_m

ROOT = Path(__file__).resolve().parents[2]
FIRMWARE = ROOT / "firmware"
WEB_FIRMWARE = ("betaflight", "none")
"""The firmwares the page can run: Betaflight (WebAssembly), and none (the truck)."""
WEB_PHYSICS = ("quad", "browser-truck")
VIDEO: tuple[VideoSystem, ...] = ("analog", "digital")


def flies_in_page(vehicle: Vehicle) -> bool:
    return vehicle.firmware.kind in WEB_FIRMWARE and vehicle.physics.kind in WEB_PHYSICS


def _grounded(location: Location) -> Location:
    """The location with its ground's height a number."""
    if location.ground_alt_m != "auto":
        return location
    ground = ground_elevation_m(location.origin[0], location.origin[1])
    return location.model_copy(update={"ground_alt_m": ground})


def web_content(content: Content) -> ContentMsg:
    return ContentMsg(
        locations={k: _grounded(v) for k, v in content.locations.items()},
        vehicles={k: v for k, v in content.vehicles.items() if flies_in_page(v)},
        courses=content.courses,
        controllers=content.controllers,
        weapons=content.weapons,
        payloads=content.payloads,
    )


def settings(vehicle: Vehicle, video: VideoSystem) -> str:
    """The vehicle's CLI settings for a video system, as Betaflight is given them: its files
    and each one's video file after it, comments and blank lines gone."""
    files = settings_files(
        [FIRMWARE / name for name in vehicle.firmware.params], video
    )
    return "\n".join(commands(load_settings(files))) + "\n"


def airframe_data(name: str) -> dict[str, Any]:
    """An airframe as the page's plant takes it: SkyFlow-Dynamics' flat parameter vector, and
    what the model around it reads (physics.quad.plant.Airframe)."""
    import numpy as np

    from drone_duels.physics.quad.plant import Plant, airframe

    frame = airframe(name)
    return {
        "params": [float(x) for x in np.asarray(Plant(frame).params)],
        "mass": frame.mass,
        "grav": frame.grav,
        "rotor_min": frame.rotor_min,
        "rotor_max": frame.rotor_max,
        "ct2": frame.ct2,
        "cq2": frame.cq2,
    }


def sim_data(content: Content) -> dict[str, Any]:
    vehicles: dict[str, Any] = {}
    airframes: dict[str, Any] = {}
    for vehicle in content.vehicles.values():
        if not flies_in_page(vehicle) or vehicle.firmware.kind != "betaflight":
            continue
        name = vehicle.physics.airframe
        if name is None:
            raise ValueError(f"vehicle {vehicle.id}: a quad needs physics.airframe")
        airframes.setdefault(name, airframe_data(name))
        vehicles[vehicle.id] = {
            "airframe": name,
            "settings": {video: settings(vehicle, video) for video in VIDEO},
        }
    return {"vehicles": vehicles, "airframes": airframes}


def write(out: Path, content: Content | None = None) -> None:
    content = content or load()
    out.mkdir(parents=True, exist_ok=True)
    (out / "content.json").write_text(web_content(content).model_dump_json())
    config = ConfigMsg(map_keys=MapKeys(), qgc_port=0, version=__version__)
    (out / "config.json").write_text(config.model_dump_json())
    (out / "sim.json").write_text(json.dumps(sim_data(content)))
    font = mcm_png(BETAFLIGHT_FONT)
    if font is None:
        raise FileNotFoundError(f"no OSD font at {BETAFLIGHT_FONT}: run scripts/setup.sh")
    (out / "osd" / "font").mkdir(parents=True, exist_ok=True)
    (out / "osd" / "font" / "betaflight.png").write_bytes(font)


# What is checked in, generated.

WEB = ROOT / "web"
FIXTURES = WEB / "test" / "fixtures"
WASM = WEB / "wasm"
FLIGHT_ORIGIN = (43.63093, -79.38945, 77.0)
FLIGHT_HEADING = 270.0
FLIGHT_EVERY = 50
"""Frames between the reference flight's samples."""


def flight_script() -> list[tuple[int, tuple[int, int, int, int]]]:
    """The reference flight's motors: (frames, ArduPilot's motors 1-4 in µs) in turn. Armed at
    idle on the pad, up, a push on each axis and back, a slow descent onto its legs, and
    stopped."""
    def push(a: tuple[int, int, int, int]) -> list[tuple[int, tuple[int, int, int, int]]]:
        """A push about an axis and back to level: one way, twice as long the other, and
        the first again, so it ends as it began but for where it has drifted."""
        base = 1290
        plus = (base + a[0], base + a[1], base + a[2], base + a[3])
        minus = (base - a[0], base - a[1], base - a[2], base - a[3])
        return [(40, plus), (80, minus), (40, plus)]

    return [
        (300, (1000, 1000, 1000, 1000)),
        (400, (1100, 1100, 1100, 1100)),
        (700, (1450, 1450, 1450, 1450)),
        *push((-30, 30, 30, -30)),  # roll: the left pair up
        *push((-30, 30, -30, 30)),  # pitch: the rear pair up
        (120, (1350, 1350, 1230, 1230)),  # yaw: the anticlockwise pair up,
        (120, (1230, 1230, 1350, 1350)),  # and stopped again
        (500, (1290, 1290, 1290, 1290)),
        (4500, (1258, 1258, 1258, 1258)),
        (600, (1000, 1000, 1000, 1000)),
    ]


def reference_flight() -> dict[str, Any]:
    """The game's quad model flown through `flight_script` in still air, a millisecond a frame
    (Betaflight's), the ground moved once on the way (as the map tells it): every
    FLIGHT_EVERY-th frame's state."""
    from drone_duels.physics.quad.model import QuadModel
    from drone_duels.physics.quad.plant import airframe

    model = QuadModel(airframe("racer5in"), FLIGHT_ORIGIN, FLIGHT_HEADING, {}, seed=0)
    samples: list[dict[str, Any]] = []
    frame = 0
    ground_at, ground_alt = 1800, FLIGHT_ORIGIN[2] - 0.4
    for frames, pwm in flight_script():
        for _ in range(frames):
            if frame == ground_at:
                model.set_ground(ground_alt)
            fdm = model.step(0.001, pwm)
            frame += 1
            if frame % FLIGHT_EVERY == 0:
                pose = model.pose()
                assert pose is not None
                samples.append({
                    "frame": frame,
                    "phase": model.phase,
                    "gyro": list(fdm.gyro),
                    "accel": list(fdm.accel),
                    "position": list(fdm.position),
                    "quat": list(fdm.quat),
                    "velocity": list(fdm.velocity),
                    "battery_v": fdm.battery_v,
                    "battery_a": fdm.battery_a,
                    "pose": list(pose),
                })
    return {
        "airframe": "racer5in",
        "origin": list(FLIGHT_ORIGIN),
        "heading_deg": FLIGHT_HEADING,
        "script": [[frames, list(pwm)] for frames, pwm in flight_script()],
        "ground": {"frame": ground_at, "alt": ground_alt},
        "samples": samples,
    }


def sfd_sources() -> dict[Path, str] | None:
    """SkyFlow-Dynamics' generated C for a quad, or None if the installed skyflow-dynamics has
    no C backend yet."""
    import importlib

    try:
        backend = importlib.import_module("skyflow_dynamics.backends.c")
    except ImportError:
        return None
    return {WASM / "sfd.c": str(backend.generate(4)), WASM / "sfd.h": str(backend.header(4))}


def generated(content: Content | None = None) -> dict[Path, str]:
    content = content or load()
    data = {
        "content": json.loads(web_content(content).model_dump_json()),
        "sim": sim_data(content),
    }
    out = {
        FIXTURES / "sim-data.json": json.dumps(data, indent=1) + "\n",
        FIXTURES / "sim-flight.json": json.dumps(reference_flight()) + "\n",
    }
    out.update(sfd_sources() or {})
    return out
