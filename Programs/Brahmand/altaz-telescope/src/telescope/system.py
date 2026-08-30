"""Top-level orchestrator: receive a celestial target, generate telescope axis

commands, and drive the (simulated) mount closed-loop for slew + tracking."""

from __future__ import annotations



from dataclasses import dataclass, field

from datetime import datetime, timedelta, timezone



from .coordinates import Observer, equatorial_to_horizontal

from .controller import PID

from .hardware import SimulatedAxis

from .mount import MountLimits, check_limits, unwrap_azimuth

from .targets import Target





@dataclass

class MountConfig:

    counts_per_rev: int = 4000          # encoder counts / motor rev

    gear_ratio: float = 400.0           # motor revs / axis rev

    max_vel_cps: float = 200_000.0      # counts/s  (~4.5 deg/s at these ratios)

    max_accel_cps2: float = 400_000.0   # counts/s^2

    encoder_noise: int = 1              # +/- counts

    # PID gains (counts/s output per count of error).

    kp: float = 8.0

    ki: float = 0.5

    kd: float = 0.05

    limits: MountLimits = field(default_factory=MountLimits)





@dataclass

class AxisCommand:

    """A single control-tick record: the commands issued to both axes."""

    t: float                 # seconds since start

    time_utc: datetime

    target_alt: float

    target_az: float         # continuous (unwrapped) setpoint

    enc_alt: float           # encoder-reported altitude

    enc_az: float

    err_alt_deg: float

    err_az_deg: float

    cmd_vel_alt_cps: float   # commanded motor velocity, counts/s

    cmd_vel_az_cps: float

    status: str



    def format_row(self) -> str:

        return (f"{self.t:6.1f}s | "

                f"tgt alt={self.target_alt:7.3f} az={self.target_az:8.3f} | "

                f"enc alt={self.enc_alt:7.3f} az={self.enc_az:8.3f} | "

                f"err alt={self.err_alt_deg:+7.4f} az={self.err_az_deg:+7.4f} | "

                f"vel alt={self.cmd_vel_alt_cps:+9.0f} az={self.cmd_vel_az_cps:+9.0f} | "

                f"{self.status}")





class TelescopeSystem:

    def __init__(self, observer: Observer, config: MountConfig | None = None):

        self.observer = observer

        self.config = config or MountConfig()

        c = self.config



        self.alt_axis = SimulatedAxis(c.counts_per_rev, c.gear_ratio,

                                      c.max_vel_cps, c.max_accel_cps2, c.encoder_noise)

        self.az_axis = SimulatedAxis(c.counts_per_rev, c.gear_ratio,

                                     c.max_vel_cps, c.max_accel_cps2, c.encoder_noise)

        self.alt_pid = PID(c.kp, c.ki, c.kd, c.max_vel_cps)

        self.az_pid = PID(c.kp, c.ki, c.kd, c.max_vel_cps)



        self._az_continuous = 0.0   # last unwrapped az setpoint



    # --- coordinate solving ---

    def solve(self, target: Target, when: datetime) -> tuple[float, float]:

        return equatorial_to_horizontal(target.ra_deg, target.dec_deg, when, self.observer)



    def _feedforward_rates(self, target: Target, when: datetime) -> tuple[float, float]:

        """Numeric d(alt)/dt and d(az)/dt in deg/s via 1s finite difference."""

        a0, z0 = self.solve(target, when)

        a1, z1 = self.solve(target, when + timedelta(seconds=1))

        # unwrap az difference across the 0/360 seam

        dz = ((z1 - z0 + 180) % 360) - 180

        return (a1 - a0), dz



    def initialize_to(self, target: Target, when: datetime) -> None:

        """Pre-seed encoders at the target so a run starts already pointed

        (skip this to simulate a full cold-start slew)."""

        alt, az = self.solve(target, when)

        az_c = unwrap_azimuth(az, 0.0, self.config.limits)

        self.alt_axis._position = self.alt_axis.deg_to_counts(alt)

        self.az_axis._position = self.az_axis.deg_to_counts(az_c)

        self._az_continuous = az_c



    # --- one control tick ---

    def tick(self, target: Target, when: datetime, t: float, dt: float) -> AxisCommand:

        alt_sp, az_raw = self.solve(target, when)

        current_az = self.az_axis.angle_deg()

        az_sp = unwrap_azimuth(az_raw, current_az, self.config.limits)

        self._az_continuous = az_sp



        status = check_limits(alt_sp, az_sp, self.config.limits)



        # Encoder feedback (this is what a real controller sees).

        enc_alt = self.alt_axis.read_encoder_counts() / self.alt_axis.counts_per_deg

        enc_az = self.az_axis.read_encoder_counts() / self.az_axis.counts_per_deg



        # Feedforward tracking rate (deg/s -> counts/s).

        ff_alt_dps, ff_az_dps = self._feedforward_rates(target, when)

        ff_alt = ff_alt_dps * self.alt_axis.counts_per_deg

        ff_az = ff_az_dps * self.az_axis.counts_per_deg



        # PID on position error (in counts).

        err_alt_counts = (alt_sp - enc_alt) * self.alt_axis.counts_per_deg

        err_az_counts = (az_sp - enc_az) * self.az_axis.counts_per_deg

        cmd_alt = self.alt_pid.update(err_alt_counts, dt) + ff_alt

        cmd_az = self.az_pid.update(err_az_counts, dt) + ff_az



        # Safety: refuse to drive below horizon or outside cable wrap.

        if status.blocked:

            cmd_alt = cmd_az = 0.0

            self.alt_axis.stop()

            self.az_axis.stop()

        else:

            self.alt_axis.command_velocity(cmd_alt)

            self.az_axis.command_velocity(cmd_az)



        # Advance the (simulated) hardware.

        self.alt_axis.step(dt)

        self.az_axis.step(dt)



        return AxisCommand(

            t=t, time_utc=when,

            target_alt=alt_sp, target_az=az_sp,

            enc_alt=enc_alt, enc_az=enc_az,

            err_alt_deg=alt_sp - enc_alt, err_az_deg=az_sp - enc_az,

            cmd_vel_alt_cps=cmd_alt, cmd_vel_az_cps=cmd_az,

            status=status.describe(),

        )



    def run(self, target: Target, start: datetime, duration_s: float,

            dt: float = 0.5, print_every: int = 1) -> list[AxisCommand]:

        """Slew to target then track for `duration_s`, printing axis commands."""

        if start.tzinfo is None:

            start = start.replace(tzinfo=timezone.utc)



        alt0, az0 = self.solve(target, start)

        print(f"Target: {target.name}  RA={target.ra_deg:.3f} deg  Dec={target.dec_deg:.3f} deg")

        print(f"Initial Alt/Az at {start.isoformat()}: alt={alt0:.3f} az={az0:.3f}\n")



        commands: list[AxisCommand] = []

        n = int(duration_s / dt)

        for i in range(n):

            when = start + timedelta(seconds=i * dt)

            cmd = self.tick(target, when, t=i * dt, dt=dt)

            commands.append(cmd)

            if i % print_every == 0:

                print(cmd.format_row())

        return commands

