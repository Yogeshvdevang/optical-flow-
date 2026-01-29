import math
import time
from typing import Tuple

from pymavlink import mavutil


def ned_to_lla(
    north_m: float,
    east_m: float,
    down_m: float,
    base_lat: float,
    base_lon: float,
    base_alt_m: float,
) -> Tuple[float, float, float]:
    # Simple flat-earth conversion for short distances.
    earth_radius_m = 6378137.0
    dlat = north_m / earth_radius_m
    dlon = east_m / (earth_radius_m * math.cos(math.radians(base_lat)))
    lat = base_lat + math.degrees(dlat)
    lon = base_lon + math.degrees(dlon)
    alt = base_alt_m - down_m
    return lat, lon, alt


def build_gps_input(
    mav: mavutil.mavlink_connection,
    lat: float,
    lon: float,
    alt_m: float,
    vx: float,
    vy: float,
    vz: float,
) -> object:
    # MAVLink expects lat/lon in 1e7, alt in meters.
    return mav.mav.gps_input_encode(
        int(time.time() * 1e6),
        0,
        0,
        0,
        0,
        int(lat * 1e7),
        int(lon * 1e7),
        alt_m,
        1.0,
        1.0,
        vx,
        vy,
        vz,
        0.0,
        0.0,
    )
