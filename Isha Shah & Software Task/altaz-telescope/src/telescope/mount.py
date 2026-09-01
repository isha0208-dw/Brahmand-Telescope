"""Alt-Az mount kinematics: azimuth cable-wrap unwrapping and safety limits.



Alt-az mounts can't spin azimuth forever (cables). The mechanical azimuth

travel is a range like [-270, +270] deg. We keep a *continuous* azimuth so the

controller drives the short way when possible, and unwraps (takes the long way)

only when a limit would otherwise be crossed.

"""

from __future__ import annotations



from dataclasses import dataclass





@dataclass(frozen=True)

class MountLimits:

    az_min_deg: float = -270.0     # cable-wrap travel

    az_max_deg: float = 270.0

    alt_min_deg: float = 5.0       # horizon / obstruction limit

    alt_max_deg: float = 89.0      # zenith keyhole: az rate diverges near 90





def unwrap_azimuth(target_az_deg: float, current_az_deg: float,

                   limits: MountLimits) -> float:

    """Return a continuous azimuth setpoint near the current position.



    Picks the representation of target_az (mod 360) closest to current_az, then

    if that lands outside cable-wrap travel, shifts by +/-360 to get back in range.

    """

    t = target_az_deg

    while t - current_az_deg > 180.0:

        t -= 360.0

    while t - current_az_deg < -180.0:

        t += 360.0



    if t > limits.az_max_deg:

        t -= 360.0            # unwrap the other way

    elif t < limits.az_min_deg:

        t += 360.0

    return t





@dataclass

class LimitStatus:

    below_horizon: bool

    zenith_keyhole: bool

    az_out_of_range: bool



    @property

    def blocked(self) -> bool:

        return self.below_horizon or self.az_out_of_range



    def describe(self) -> str:

        flags = []

        if self.below_horizon:

            flags.append("BELOW-HORIZON")

        if self.zenith_keyhole:

            flags.append("ZENITH-KEYHOLE")

        if self.az_out_of_range:

            flags.append("AZ-LIMIT")

        return ",".join(flags) if flags else "OK"





def check_limits(alt_deg: float, az_continuous_deg: float,

                 limits: MountLimits) -> LimitStatus:

    return LimitStatus(

        below_horizon=alt_deg < limits.alt_min_deg,

        zenith_keyhole=alt_deg > limits.alt_max_deg,

        az_out_of_range=not (limits.az_min_deg <= az_continuous_deg <= limits.az_max_deg),

    )