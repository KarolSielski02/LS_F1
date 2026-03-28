from __future__ import annotations

from dataclasses import dataclass, field

# Throttle on a straight (low steer)
CRUISE_THROTTLE: float = 0.7

# Minimum throttle allowed at full lock (abs(steer) == 1.0)
MIN_CORNER_THROTTLE: float = 0.05

# Steer magnitude below which full cruise throttle is kept
STEER_THRESHOLD: float = 0.05  # tighter — minor corrections don't count as "straight"

# How fast throttle ramps up per tick toward target (~2s to full at 60 Hz)
THROTTLE_RAMP_RATE: float = 0.02

# How fast throttle ramps down per tick toward a lower target (corners/slip)
THROTTLE_DROP_RATE: float = 0.04

# How much to back off per tick when wheel slip is detected
SLIP_REDUCTION: float = 0.10  # stronger reduction — one slip event has real impact

# Tire temperature below which throttle is reduced (cold tires = low grip)
TIRE_COLD_THRESHOLD_C: float = 50.0

# Maximum throttle allowed when tires are fully cold
TIRE_COLD_MAX_THROTTLE: float = 0.35


def _tire_temp_multiplier(avg_temp_celsius: float) -> float:
    """Scale max throttle based on average tire temperature.

    - Below TIRE_COLD_THRESHOLD_C: capped at TIRE_COLD_MAX_THROTTLE
    - Above threshold: full throttle allowed (multiplier = 1.0)
    - Between: linear ramp from cold cap to 1.0
    """
    if avg_temp_celsius >= TIRE_COLD_THRESHOLD_C:
        return 1.0
    return TIRE_COLD_MAX_THROTTLE + (1.0 - TIRE_COLD_MAX_THROTTLE) * (avg_temp_celsius / TIRE_COLD_THRESHOLD_C)


def _target_from_steer(steer: float) -> float:
    """Map steering magnitude to a throttle target.

    - |steer| <= STEER_THRESHOLD  → CRUISE_THROTTLE (straight)
    - |steer| == 1.0              → MIN_CORNER_THROTTLE (full lock)
    - Between                     → linear interpolation
    """
    magnitude = abs(steer)

    if magnitude <= STEER_THRESHOLD:
        return CRUISE_THROTTLE

    # Normalize from [STEER_THRESHOLD, 1.0] → [0.0, 1.0]
    t = (magnitude - STEER_THRESHOLD) / (1.0 - STEER_THRESHOLD)
    return CRUISE_THROTTLE + t * (MIN_CORNER_THROTTLE - CRUISE_THROTTLE)


@dataclass
class ThrottleController:
    """Stateful throttle that smoothly tracks a steer-aware target."""

    _current: float = field(default=0.0, init=False)

    def tick(
        self,
        steer: float,
        max_slip: float,
        slip_threshold: float,
        avg_tire_temp: float,
        is_braking: bool,
    ) -> float:
        """Advance one tick and return the throttle value to apply.

        - While braking: forced to 0.0, ramp paused.
        - Cold tires: target capped to prevent spin on low-grip surface.
        - Sharp steer: target drops toward MIN_CORNER_THROTTLE.
        - Wheel slip: additional gentle back-off on top of steer reduction.
        - Straight + warm tires: ramps back up to CRUISE_THROTTLE.
        """
        if is_braking:
            self._current = max(0.0, self._current - THROTTLE_DROP_RATE)
            return 0.0

        target = _target_from_steer(steer)

        # Cap target based on tire temperature — cold tires have less grip
        temp_mult = _tire_temp_multiplier(avg_tire_temp)
        target = target * temp_mult

        if max_slip > slip_threshold:
            target = max(0.0, target - SLIP_REDUCTION)

        if self._current < target:
            self._current = min(target, self._current + THROTTLE_RAMP_RATE)
        elif self._current > target:
            self._current = max(target, self._current - THROTTLE_DROP_RATE)

        return round(self._current, 4)

    def reset(self, value: float = 0.0) -> None:
        """Manually override the current throttle level."""
        self._current = max(0.0, min(1.0, value))

    @property
    def current(self) -> float:
        return self._current
