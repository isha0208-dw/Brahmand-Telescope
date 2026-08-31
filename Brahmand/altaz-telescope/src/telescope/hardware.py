"""Hardware abstraction: motor-controller and encoder interfaces, plus a

simulated axis that models real drive physics (accel/velocity limits) and a

quantized, noisy position encoder.



Swap SimulatedAxis for a real driver by implementing the same

`command_velocity` / `read_encoder_counts` methods.

"""

from __future__ import annotations



import random

from abc import ABC, abstractmethod





class MotorController(ABC):

    @abstractmethod

    def command_velocity(self, counts_per_sec: float) -> None: ...

    @abstractmethod

    def stop(self) -> None: ...





class Encoder(ABC):

    @abstractmethod

    def read_encoder_counts(self) -> int: ...





class SimulatedAxis(MotorController, Encoder):

    """One mount axis: motor + gearbox + shaft encoder, simulated.



    counts_per_rev  : encoder counts per motor-shaft revolution

    gear_ratio      : motor revolutions per axis revolution (reduction)

    max_vel_cps     : max motor velocity, encoder counts/second

    max_accel_cps2  : max motor acceleration, counts/second^2

    encoder_noise   : +/- counts of uniform read noise (quantization jitter)

    """



    def __init__(self, counts_per_rev: int, gear_ratio: float,

                 max_vel_cps: float, max_accel_cps2: float,

                 encoder_noise: int = 0):

        self.counts_per_axis_rev = counts_per_rev * gear_ratio

        self.counts_per_deg = self.counts_per_axis_rev / 360.0

        self.max_vel_cps = max_vel_cps

        self.max_accel_cps2 = max_accel_cps2

        self.encoder_noise = encoder_noise



        self._position = 0.0        # true motor position (counts, float)

        self._velocity = 0.0        # true current velocity (counts/s)

        self._target_velocity = 0.0  # commanded velocity (counts/s)



    # --- MotorController ---

    def command_velocity(self, counts_per_sec: float) -> None:

        self._target_velocity = max(-self.max_vel_cps, min(self.max_vel_cps, counts_per_sec))



    def stop(self) -> None:

        self._target_velocity = 0.0



    # --- Encoder ---

    def read_encoder_counts(self) -> int:

        noise = random.uniform(-self.encoder_noise, self.encoder_noise) if self.encoder_noise else 0.0

        return int(round(self._position + noise))



    # --- Simulation step (called by the control loop) ---

    def step(self, dt: float) -> None:

        """Advance physics by dt seconds, respecting acceleration limits."""

        dv = self._target_velocity - self._velocity

        max_dv = self.max_accel_cps2 * dt

        dv = max(-max_dv, min(max_dv, dv))

        self._velocity += dv

        self._position += self._velocity * dt



    # --- Helpers for the orchestrator ---

    def angle_deg(self) -> float:

        return self._position / self.counts_per_deg



    def deg_to_counts(self, deg: float) -> float:

        return deg * self.counts_per_deg