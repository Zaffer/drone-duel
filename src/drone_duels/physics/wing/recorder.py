"""Every flight of the wing, to a CSV file in logs/flights/, to look into how it flew."""

import csv
import math
import threading
from pathlib import Path

from drone_duels.bridges.xplane import YOKES, XPlaneServer

RECORD_HZ = 50


class FlightRecorder:
    """The flight, RECORD_HZ times a second, to `path`: the wing's true state (degrees, m, m/s,
    g), the elevons (trailing edge down +), INAV's outputs (the motor 0..1, the servos as
    X-Plane's yoke ratios), the eight RC channels INAV gets (µs) and the wind (NED, m/s).
    What INAV makes of it (armed, its own attitude) is the firmware's: telemetry, not here."""

    FIELDS = (
        "t phase lat lon alt agl roll pitch heading p q r vn ve vd airspeed alpha beta "
        "nx ny nz motor servo1 servo2 elevon_l elevon_r rpm pack_v pack_a "
        "rc1 rc2 rc3 rc4 rc5 rc6 rc7 rc8 wind_n wind_e wind_d"
    ).split()

    def __init__(self, server: XPlaneServer, path: Path) -> None:
        self.server = server
        self.path = path
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="flight-recorder", daemon=True)

    def start(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2)

    def _run(self) -> None:
        with self.path.open("w", newline="") as f:
            out = csv.writer(f)
            out.writerow(self.FIELDS)
            n = 0
            while not self._stop.wait(1 / RECORD_HZ):
                out.writerow(self.row())
                n += 1
                if n % RECORD_HZ == 0:
                    f.flush()

    def row(self) -> list[object]:
        server = self.server
        ac = server.aircraft
        s = server.snapshot()
        g = ac.gravity
        rc = [*server.sticks, *[math.nan] * 8][:8]
        values = [
            server.time,
            s.phase,
            s.lat,
            s.lon,
            s.alt,
            s.agl,
            s.roll,
            s.pitch,
            s.heading,
            *(math.degrees(w) for w in ac.body.rates),
            *s.velocity,
            s.airspeed,
            math.degrees(ac.air.alpha),
            math.degrees(ac.air.beta),
            *(-f / g for f in ac.specific_force),
            s.motor,
            server.outputs.get(YOKES[0], 0.0),
            server.outputs.get(YOKES[1], 0.0),
            *s.elevons,
            ac.power.omega * 60 / (2 * math.pi),
            s.pack_volts,
            s.pack_amps,
            *rc,
            *ac.wind,
        ]
        return [round(v, 4) if isinstance(v, float) else v for v in values]
