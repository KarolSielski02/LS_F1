from __future__ import annotations

MAX_THROTTLE = 0.55

# Bazowy hamulec wg nasilenia zakrętu (1=łagodny, 2=średni, 3=ostry)
_BRAKE_BASE = {
    1: 0.2,
    2: 0.45,
    3: 0.7,
}

# Dodatkowy hamulec na każde 50 km/h powyżej prędkości odniesienia, wg nasilenia
_BRAKE_SPEED_GAIN = {
    1: 0.05,
    2: 0.12,
    3: 0.35,
}

_BRAKE_REF_SPEED_KMH = 80.0

# Poniżej tej prędkości dla danego zakrętu hamowanie jest wyłączone (auto już wystarczająco wolne)
_CORNER_TARGET_SPEED_KMH = {
    1: 100,  # łagodny
    2: 70,   # średni
    3: 40,   # ostry
}

# Maks. liczba kolejnych ticków z hamowaniem na podejście do zakrętu (wg nasilenia)
_MAX_BRAKE_TICKS = {
    1: 8,
    2: 15,
    3: 30,
}


class ThrottleController:
    def __init__(self) -> None:
        self._brake_ticks = 0   # ile ticków z rzędu trzyma hamulec
        self._last_severity: int | None = None

    def compute(
        self,
        speed_kmh: float,
        is_turn: bool = False,
        severity: int | None = None,
    ) -> tuple[float, float]:
        """Zwraca (gaz, hamulec). Czas hamowania jest ograniczony tickami, żeby auto nie zatrzymało się w zakręcie."""

        if not is_turn or severity is None:
            # prosta — zeruj licznik hamowania
            self._brake_ticks = 0
            self._last_severity = None
            return MAX_THROTTLE, 0.0

        # Wystarczająco wolno na ten zakręt — bez hamowania
        target_speed = _CORNER_TARGET_SPEED_KMH.get(severity, 0)
        if speed_kmh <= target_speed:
            self._brake_ticks = 0
            self._last_severity = severity
            return MAX_THROTTLE * 0.5, 0.0

        # reset przy wejściu w inny zakręt / inne nasilenie
        if severity != self._last_severity:
            self._brake_ticks = 0
            self._last_severity = severity

        max_ticks = _MAX_BRAKE_TICKS.get(severity, 20)

        if self._brake_ticks >= max_ticks:
            # okno hamowania skończone — toczenie przez zakręt
            return MAX_THROTTLE * 0.5, 0.0

        base_brake   = _BRAKE_BASE.get(severity, 0.0)
        speed_excess = max(0.0, speed_kmh - _BRAKE_REF_SPEED_KMH)
        speed_bonus  = (speed_excess / 50.0) * _BRAKE_SPEED_GAIN.get(severity, 0.0)
        brake        = min(1.0, base_brake + speed_bonus)

        self._brake_ticks += 1

        # gaz i hamulec naraz — nie
        throttle = 0.0 if brake > 0.0 else MAX_THROTTLE

        return throttle, brake
