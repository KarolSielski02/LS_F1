from __future__ import annotations

START_THROTTLE = 0.37

_MAX_ENTRY_SPEED_MIN_KMH = 45.0
_MAX_ENTRY_SPEED_MAX_KMH = 160.0

# Optymalne zakresy temperatur opon [°C]
_TIRE_TEMP_OPTIMAL = {
    "HARD": (70.0, 110.0),
    "SOFT": (50.0, 90.0),
    "WET":  (30.0, 70.0),
}
_TIRE_TEMP_DEFAULT = (60.0, 100.0)


def _max_entry_speed(intensity: float) -> float:
    return _MAX_ENTRY_SPEED_MAX_KMH - intensity * (_MAX_ENTRY_SPEED_MAX_KMH - _MAX_ENTRY_SPEED_MIN_KMH)


def _tire_grip_factor(
    min_temp: float,
    tire_type_name: str = "SOFT",
) -> float:
    """Zwraca współczynnik przyczepności [0.6–1.0] na podstawie temperatury opon.

    Poniżej optimum i powyżej optimum — redukuj prędkość.
    W optimum — pełna przyczepność (1.0).
    """
    low, high = _TIRE_TEMP_OPTIMAL.get(tire_type_name, _TIRE_TEMP_DEFAULT)

    if min_temp < low:
        # zimne opony — liniowo od 0.6 przy 0°C do 1.0 przy low
        factor = 0.6 + 0.4 * (min_temp / low)
    elif min_temp > high:
        # przegrzane — liniowo od 1.0 przy high do 0.7 przy high+40°C
        factor = 1.0 - 0.3 * min((min_temp - high) / 40.0, 1.0)
    else:
        factor = 1.0

    return max(0.6, min(1.0, factor))


class ThrottleController:
    def __init__(
        self,
        max_throttle: float = 0.75,
        speed_straight_kmh: float = 250.0,
        speed_min_kmh: float = 45.0,
        throttle_update_every: int = 5,
    ) -> None:
        self._brake_ticks = 0
        self._last_severity: int | None = None
        self._prevThrottle = START_THROTTLE
        self._maxThrottle = max_throttle
        self._last_brake = 0.0
        self._last_throttle = START_THROTTLE
        self._throttle_tick = 0
        self._throttle_update_every = throttle_update_every
        self._speed_straight = speed_straight_kmh
        self._speed_min = speed_min_kmh

    def _throttle_step(self, intensity: float) -> float:
        return 0.04 - intensity * 0.03

    def _ramp_throttle(self, target: float, intensity: float) -> float:
        self._throttle_tick += 1
        if self._throttle_tick < self._throttle_update_every:
            return self._last_throttle
        self._throttle_tick = 0
        return min(target, self._last_throttle + self._throttle_step(intensity))

    def _predictive_brake(
        self,
        speed_kmh: float,
        future_intensity: float,
        grip: float,
    ) -> float | None:
        if future_intensity < 0.1:
            return None
        future_limit = _max_entry_speed(future_intensity) * grip
        safety_buffer = 40.0 + future_intensity * 30.0
        if speed_kmh <= future_limit + safety_buffer:
            return None
        overspeed = speed_kmh - (future_limit + safety_buffer)
        return min(0.6, (overspeed / 40.0) * future_intensity)

    def compute(
        self,
        speed_kmh: float,
        intensity: float,
        future_intensity: float = 0.0,
        min_tire_temp: float = 80.0,
        tire_type_name: str = "SOFT",
    ) -> tuple[float, float]:

        grip = _tire_grip_factor(min_tire_temp, tire_type_name)

        # --- predictive braking ---
        predictive = self._predictive_brake(speed_kmh, future_intensity, grip)
        if predictive is not None:
            brake = min(predictive, self._last_brake + 0.15)
            self._last_brake = brake
            self._last_throttle = 0.0
            self._throttle_tick = 0
            return 0.0, brake

        # --- prosta ---
        if intensity < 0.05:
            self._brake_ticks = 0
            self._last_brake = 0.0
            throttle = self._ramp_throttle(self._maxThrottle, 0.0)
            self._last_throttle = throttle
            return throttle, 0.0

        # prędkość docelowa skalowana przez grip
        target_speed   = (self._speed_straight - intensity * (self._speed_straight - self._speed_min)) * grip
        coast_throttle = self._maxThrottle * (1.0 - intensity * 0.85)
        entry_limit    = _max_entry_speed(intensity) * grip  # <- grip redukuje limit wejścia

        # --- za szybko na wejście ---
        if speed_kmh > entry_limit:
            overspeed_ratio = min(1.0, (speed_kmh - entry_limit) / 50.0)
            target_brake = min(1.0, intensity * 0.85 + overspeed_ratio * 0.5)
            brake = min(target_brake, self._last_brake + 0.15)
            self._last_brake = brake
            self._last_throttle = 0.0
            self._throttle_tick = 0
            self._brake_ticks += 1
            return 0.0, brake

        # --- wystarczająco wolno ---
        if speed_kmh <= target_speed:
            self._brake_ticks = 0
            self._last_brake = 0.0
            exit_throttle = coast_throttle * (1.0 - intensity * 0.3)  # dodaj
            throttle = self._ramp_throttle(exit_throttle, intensity)  # było coast_throttle
            self._last_throttle = throttle
            return throttle, 0.0

        # --- okno hamowania ---
        max_ticks = int(5 + intensity * 50)
        if self._brake_ticks >= max_ticks:
            self._last_brake = 0.0
            throttle = self._ramp_throttle(coast_throttle, intensity)
            self._last_throttle = throttle
            return throttle, 0.0

        base_brake   = intensity * 0.85 + min(0.15, speed_kmh / 300.0)
        speed_excess = max(0.0, speed_kmh - 60.0)
        speed_bonus  = (speed_excess / 40.0) * intensity * 0.5
        target_brake = min(1.0, base_brake + speed_bonus)

        brake = min(target_brake, self._last_brake + 0.15)
        self._last_brake = brake
        self._last_throttle = 0.0
        self._throttle_tick = 0
        self._brake_ticks += 1

        return 0.0, brake