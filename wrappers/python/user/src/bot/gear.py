from __future__ import annotations

from hackarena3 import GearShift

# Gear profile: gear -> (min_speed_kmh, max_speed_kmh, rpm_downshift, rpm_upshift)
# Speed ranges overlap intentionally to add hysteresis and avoid ping-pong.
GEAR_PROFILE: dict[int, tuple[float, float, float, float]] = {
    1: (  0,  45, 2000, 5500),
    2: ( 35,  75, 2800, 5600),
    3: ( 65, 110, 3000, 5700),
    4: (100, 145, 3000, 5700),
    5: (135, 175, 3000, 5800),
    6: (165, 205, 3000, 5800),
    7: (195, 235, 3000, 5800),
    8: (225, 999, 3000, 5800),
}

# Minimum time between any two gear changes
SHIFT_COOLDOWN_MS: int = 600


def compute_gear_shift(
    gear: int,
    speed_kmh: float,
    engine_rpm: float,
    throttle: float,
    brake: float,
    now_ms: int,
    last_shift_ms: int,
) -> GearShift:
    """Gear logic locked to driving direction:
    - Accelerating (throttle > 0, no brake) → upshift only
    - Decelerating (brake > 0 or throttle == 0) → downshift only
    This prevents RPM drop after an upshift from immediately triggering a downshift.
    """
    if now_ms - last_shift_ms < SHIFT_COOLDOWN_MS:
        return GearShift.NONE

    if gear not in GEAR_PROFILE:
        return GearShift.NONE

    min_spd, max_spd, rpm_down, rpm_up = GEAR_PROFILE[gear]

    is_accelerating = throttle > 0.05 and brake < 0.05
    is_decelerating = brake > 0.05 or throttle < 0.05

    if is_accelerating:
        # Only upshift — protect rev limiter, or speed exceeded gear's range
        if engine_rpm > rpm_up and gear < 8:
            return GearShift.UPSHIFT
        if speed_kmh > max_spd and gear < 8:
            return GearShift.UPSHIFT

    if is_decelerating:
        # Only downshift — avoid engine lugging, or speed dropped below gear's range
        if engine_rpm < rpm_down and gear > 1:
            return GearShift.DOWNSHIFT
        if speed_kmh < min_spd and gear > 1:
            return GearShift.DOWNSHIFT

    return GearShift.NONE
