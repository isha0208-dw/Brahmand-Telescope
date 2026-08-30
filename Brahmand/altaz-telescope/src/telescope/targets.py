"""Celestial target definitions and a small built-in catalog."""

from __future__ import annotations



from dataclasses import dataclass





@dataclass(frozen=True)

class Target:

    """A celestial target in equatorial coordinates (J2000, degrees)."""

    name: str

    ra_deg: float      # right ascension, 0..360

    dec_deg: float     # declination, -90..90





def from_ra_hours(name: str, ra_h: float, ra_m: float, ra_s: float,

                  dec_deg: float, dec_m: float = 0.0, dec_s: float = 0.0) -> Target:

    """Build a Target from RA in h/m/s and Dec in deg/arcmin/arcsec.



    Dec sign is taken from dec_deg (use -0.0 style by passing a negative degree).

    """

    ra = (ra_h + ra_m / 60 + ra_s / 3600) * 15.0            # 1h = 15 deg

    sign = -1.0 if dec_deg < 0 else 1.0

    dec = sign * (abs(dec_deg) + dec_m / 60 + dec_s / 3600)

    return Target(name, ra % 360.0, dec)





# A few bright, well-known targets (approximate J2000 coordinates).

CATALOG: dict[str, Target] = {

    "vega":       from_ra_hours("Vega",       18, 36, 56,  38, 47, 1),

    "polaris":    from_ra_hours("Polaris",     2, 31, 49,  89, 15, 51),

    "sirius":     from_ra_hours("Sirius",      6, 45,  9, -16, 42, 58),

    "betelgeuse": from_ra_hours("Betelgeuse",  5, 55, 10,   7, 24, 25),

    "arcturus":   from_ra_hours("Arcturus",   14, 15, 40,  19, 10, 56),

    "antares":    from_ra_hours("Antares",    16, 29, 24, -26, 25, 55),

    "m31":        from_ra_hours("Andromeda Galaxy", 0, 42, 44, 41, 16, 9),

}





def from_catalog(name: str) -> Target:

    key = name.strip().lower()

    if key not in CATALOG:

        raise KeyError(f"Unknown target '{name}'. Known: {', '.join(sorted(CATALOG))}")

    return CATALOG[key]
