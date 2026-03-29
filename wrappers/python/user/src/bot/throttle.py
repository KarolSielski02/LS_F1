from __future__ import annotations

from hackarena3 import TireTemperaturePerWheel, TireType

START_THROTTLE = 0.37
DEFAULT_CORNER_THROTTLE = 0.1

# Bazowy hamulec wg nasilenia zakrętu (1=łagodny, 2=średni, 3=ostry)
_BRAKE_BASE = {
    1: 0.1,
    2: 0.16,
    3: 0.25,
    4: 0.38
}

# Dodatkowy hamulec na każde 50 km/h powyżej prędkości odniesienia, wg nasilenia
_BRAKE_SPEED_GAIN = {
    1: 0.07,
    2: 0.15,
    3: 0.25,
    4: 0.45
}

# Poniżej tej prędkości dla danego zakrętu hamowanie jest wyłączone (auto już wystarczająco wolne)
_CORNER_TARGET_SPEED_KMH = {
    1: 115,  # łagodny
    2: 85,   # średni
    3: 70,   # ostry
    4: 54
}

_CORNER_THROTTLE_MULT = {
    1: 0.8,
    2: 0.65,
    3: 0.45,
    4: 0.35
}

_MIN_SPEED = 30.0

# Maks. liczba kolejnych ticków z hamowaniem na podejście do zakrętu (wg nasilenia)
_MAX_BRAKE_TICKS = {
    1: 8,
    2: 12,
    3: 18,
}

def get_max_speed_and_brake_addition(tireTemps: TireTemperaturePerWheel, tireType: TireType):
    minTemp = min(tireTemps.front_left_celsius,tireTemps.front_right_celsius,tireTemps.rear_left_celsius,tireTemps.rear_right_celsius)
    if tireType == TireType.HARD:
        if minTemp < 65.0:
            return 0.6, 0.2
        if minTemp >= 65.0:
            return 0.83, 0.25
    else: 
        if minTemp < 50.0:
            return 0.6, 0.2
        if minTemp >= 50:
            return 0.83, 0.25   


def is_steering(steer: float):
    return steer>=-0.15 and steer<=0.15

def should_use_gas(steer: float, currSpeed: float, severity: int) -> bool:
    steerInRange = steer>=-0.15 and steer<=0.15
    if severity is not None:
        return steerInRange

    return currSpeed < _MIN_SPEED

class ThrottleController:
    def __init__(self) -> None:
        self._brake_ticks = 0   # ile ticków z rzędu trzyma hamulec
        self._last_severity: int | None = None

        self._maxThrottle = 60.0
        self._brakeAddition = 0.2

    def compute(
        self,
        speed_kmh: float,
        tireTemps: TireTemperaturePerWheel, 
        tireType: TireType,
        currentSteer: float,
        tick: int,
        is_turn: bool = False,
        severity: int | None = None
    ) -> tuple[float, float, float]:
        """Zwraca (gaz, hamulec). Czas hamowania jest ograniczony tickami, żeby auto nie zatrzymało się w zakręcie."""
        maxThrottle, brakeAddition = get_max_speed_and_brake_addition(tireTemps, tireType)
        self._maxThrottle = maxThrottle
        self._brakeAddition = brakeAddition

        if not is_turn or severity is None:
            return self._maxThrottle, 0.0, 0.5

        # Wystarczająco wolno na ten zakręt — bez hamowania
        target_speed = _CORNER_TARGET_SPEED_KMH.get(severity, 0)
        if speed_kmh <= target_speed:
            cornerThrottleMult = _CORNER_THROTTLE_MULT.get(severity, DEFAULT_CORNER_THROTTLE)
            return cornerThrottleMult*self._maxThrottle, 0.0, 0.5
        if speed_kmh > target_speed and speed_kmh <= target_speed+20.0:
            return 0.0, 0.0, 0.5

        base_brake   = _BRAKE_BASE.get(severity, 0.0)
        speed_excess = max(0.0, speed_kmh - target_speed)
        speed_bonus  = (speed_excess / 50.0) * _BRAKE_SPEED_GAIN.get(severity, 0.0)
        brake        = min(1.0, base_brake + speed_bonus + self._brakeAddition)

        # gaz i hamulec naraz — nie
        throttle = 0.0 if brake > 0.0 else START_THROTTLE

        return throttle, brake, 0.44
