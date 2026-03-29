from __future__ import annotations

from hackarena3 import GearShift

_UPSHIFT_RPM   = 12500
_DOWNSHIFT_RPM = 2000

# Minimalna prędkość (km/h) utrzymania biegu — poniżej redukcja
_GEAR_MIN_SPEED_KMH = {
    2: 25,
    3: 45,
    4: 70,
    5: 200,
    6: 250,
    7: 260,
    8: 300,
}

# Minimalna liczba ticków między kolejnymi zmianami biegu
_SHIFT_COOLDOWN_TICKS = 20


class GearController:
    def __init__(self) -> None:
        self._ticks_since_shift = _SHIFT_COOLDOWN_TICKS  # od razu można zmienić bieg

    def compute(self, gear: int, engine_rpm: float, speed_kmh: float = 0.0) -> GearShift:
        """Zwraca komendę skrzyni z zachowaniem cooldownu między zmianami."""
        self._ticks_since_shift += 1

        if self._ticks_since_shift < _SHIFT_COOLDOWN_TICKS:
            return GearShift.NONE

        # Wysokie obroty — wyższy bieg
        if engine_rpm > _UPSHIFT_RPM and gear < 8:
            self._ticks_since_shift = 0
            return GearShift.UPSHIFT

        # Niski bieg: za niskie RPM albo za mała prędkość na aktualny bieg
        rpm_too_low   = engine_rpm < _DOWNSHIFT_RPM and gear > 1
        speed_too_low = speed_kmh < _GEAR_MIN_SPEED_KMH.get(gear, 0) and gear > 1

        if rpm_too_low or speed_too_low:
            self._ticks_since_shift = 0
            return GearShift.DOWNSHIFT

        return GearShift.NONE
