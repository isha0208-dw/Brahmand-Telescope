"""Per-axis closed-loop control: PID with anti-windup, output-clamped to the

axis velocity limit. Produces a motor velocity command (encoder counts/sec)."""

from __future__ import annotations





class PID:

    def __init__(self, kp: float, ki: float, kd: float, out_limit: float):

        self.kp, self.ki, self.kd = kp, ki, kd

        self.out_limit = out_limit

        self._integral = 0.0

        self._prev_error = 0.0

        self._initialized = False



    def reset(self) -> None:

        self._integral = 0.0

        self._prev_error = 0.0

        self._initialized = False



    def update(self, error: float, dt: float) -> float:

        if dt <= 0:

            return 0.0

        derivative = 0.0 if not self._initialized else (error - self._prev_error) / dt

        self._prev_error = error

        self._initialized = True



        self._integral += error * dt

        out = self.kp * error + self.ki * self._integral + self.kd * derivative



        # Clamp and anti-windup: unwind integral if we're saturated.

        if out > self.out_limit:

            self._integral -= (out - self.out_limit) / self.ki if self.ki else 0.0

            out = self.out_limit

        elif out < -self.out_limit:

            self._integral -= (out + self.out_limit) / self.ki if self.ki else 0.0

            out = -self.out_limit

        return out