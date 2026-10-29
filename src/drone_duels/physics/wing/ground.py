"""The ground under the wing, as the page's map has it: its 3D tiles' roofs and trees too."""

import math
from collections import deque

EARTH_RADIUS = 6378137.0


class MapGround:
    """The ground's height above sea level under the wing, as the map has it: the page reports
    it where the wing is about to be, a little ahead, so a report is late by the time the wing
    gets there. The ground at a place is the latest report within NEAR_M of it, or else the
    latest of all; at the start, the start's. Flat at the origin's height until a page
    reports."""

    NEAR_M = 4.0
    KEPT = 64

    def __init__(self, origin: tuple[float, float, float]) -> None:
        self.origin = origin
        self.reports: deque[tuple[float, float, float]] = deque(maxlen=self.KEPT)
        self.home: float | None = None

    def report(self, lat: float, lon: float, alt: float) -> None:
        self.reports.append((lat, lon, alt))
        if self._metres(lat, lon, *self.origin[:2]) < self.NEAR_M:
            self.home = alt

    def _metres(self, lat0: float, lon0: float, lat1: float, lon1: float) -> float:
        north = math.radians(lat1 - lat0) * EARTH_RADIUS
        east = math.radians(lon1 - lon0) * EARTH_RADIUS * math.cos(math.radians(lat0))
        return math.hypot(north, east)

    def __call__(self, lat: float, lon: float) -> float:
        for r_lat, r_lon, alt in reversed(self.reports):
            if self._metres(lat, lon, r_lat, r_lon) < self.NEAR_M:
                return alt
        if self.home is not None and self._metres(lat, lon, *self.origin[:2]) < self.NEAR_M:
            return self.home
        return self.reports[-1][2] if self.reports else self.origin[2]
