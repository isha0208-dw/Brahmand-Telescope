"""Celestial coordinate transforms: equatorial (RA/Dec) -> horizontal (Alt/Az).



All angles in degrees unless noted. Azimuth is measured from North (0 deg),

increasing toward East (90 deg = East), which matches most mount conventions.

"""

from __future__ import annotations



import math

from dataclasses import dataclass

from datetime import datetime, timezone





@dataclass(frozen=True)

class Observer:

    """Geographic location of the telescope."""

    latitude_deg: float          # + north

    longitude_deg: float         # + east

    elevation_m: float = 0.0





def julian_date(dt: datetime) -> float:

    """Julian Date for a timezone-aware datetime."""

    if dt.tzinfo is None:

        dt = dt.replace(tzinfo=timezone.utc)

    dt = dt.astimezone(timezone.utc)



    y, m = dt.year, dt.month

    day = dt.day + (dt.hour + (dt.minute + (dt.second + dt.microsecond / 1e6) / 60) / 60) / 24

    if m <= 2:

        y -= 1

        m += 12

    a = y // 100

    b = 2 - a + a // 4

    return math.floor(365.25 * (y + 4716)) + math.floor(30.6001 * (m + 1)) + day + b - 1524.5





def gmst_deg(dt: datetime) -> float:

    """Greenwich Mean Sidereal Time in degrees."""

    d = julian_date(dt) - 2451545.0

    return (280.46061837 + 360.98564736629 * d) % 360.0





def local_sidereal_deg(dt: datetime, observer: Observer) -> float:

    """Local Sidereal Time in degrees for the observer."""

    return (gmst_deg(dt) + observer.longitude_deg) % 360.0





def equatorial_to_horizontal(ra_deg: float, dec_deg: float,

                             dt: datetime, observer: Observer) -> tuple[float, float]:

    """Convert RA/Dec (degrees) to (altitude_deg, azimuth_deg) at time dt.



    Returns altitude in [-90, 90] and azimuth in [0, 360).

    """

    lst = local_sidereal_deg(dt, observer)

    hour_angle = math.radians((lst - ra_deg) % 360.0)

    dec = math.radians(dec_deg)

    lat = math.radians(observer.latitude_deg)



    sin_alt = math.sin(dec) * math.sin(lat) + math.cos(dec) * math.cos(lat) * math.cos(hour_angle)

    sin_alt = max(-1.0, min(1.0, sin_alt))

    alt = math.asin(sin_alt)



    # North-referenced, East-positive azimuth.

    y = -math.cos(dec) * math.sin(hour_angle)

    x = math.sin(dec) * math.cos(lat) - math.cos(dec) * math.sin(lat) * math.cos(hour_angle)

    az = math.degrees(math.atan2(y, x)) % 360.0



    return math.degrees(alt), az

