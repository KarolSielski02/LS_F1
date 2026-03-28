from __future__ import annotations

from dataclasses import dataclass, field

# How fast brake pressure builds up and bleeds off per tick
BRAKE_RAMP_RATE: float = 0.04   # ~0.7s to full brake at 60 Hz
BRAKE_RELEASE_RATE: float = 0.06  # slightly faster release than application


@dataclass
class BrakeController:
    """Stateful smooth braking — call request() to set target, tick() each frame."""

    _current: float = field(default=0.0, init=False)
    _target: float = field(default=0.0, init=False)

    def request(self, target: float) -> None:
        """Set desired brake level [0.0 – 1.0]. Call once per decision."""
        self._target = max(0.0, min(1.0, target))

    def release(self) -> None:
        """Smoothly release brakes back to zero."""
        self._target = 0.0

    def tick(self) -> tuple[float, float]:
        """Advance one simulation tick.

        Returns (brake, throttle):
        - brake ramps toward target
        - throttle is 0.0 while any brake pressure is applied, 1.0 otherwise
        """
        if self._target > self._current:
            self._current = min(self._target, self._current + BRAKE_RAMP_RATE)
        else:
            self._current = max(self._target, self._current - BRAKE_RELEASE_RATE)

        throttle_allowed = 0.0 if self._current > 0.01 else 1.0
        return round(self._current, 4), throttle_allowed

    @property
    def is_braking(self) -> bool:
        return self._current > 0.01

    @property
    def current(self) -> float:
        return self._current
