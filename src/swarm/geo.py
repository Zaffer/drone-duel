"""Flat-earth conversion between WGS84 lat/lon and a local north-east frame."""

import math
from dataclasses import dataclass

EARTH_RADIUS_M = 6_378_137.0


@dataclass(frozen=True)
class LocalFrame:
    """A local tangent plane anchored at an origin. Valid to a few kilometres."""

    origin_lat_deg: float
    origin_lon_deg: float

    def to_ne(self, lat_deg: float, lon_deg: float) -> tuple[float, float]:
        d_lat = math.radians(lat_deg - self.origin_lat_deg)
        d_lon = math.radians(lon_deg - self.origin_lon_deg)
        north = d_lat * EARTH_RADIUS_M
        east = d_lon * EARTH_RADIUS_M * math.cos(math.radians(self.origin_lat_deg))
        return north, east

    def to_latlon(self, north_m: float, east_m: float) -> tuple[float, float]:
        lat = self.origin_lat_deg + math.degrees(north_m / EARTH_RADIUS_M)
        lon = self.origin_lon_deg + math.degrees(
            east_m / (EARTH_RADIUS_M * math.cos(math.radians(self.origin_lat_deg)))
        )
        return lat, lon
