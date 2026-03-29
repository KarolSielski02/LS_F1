from __future__ import annotations

# Blokada dyferencjału per severity w środku zakrętu
_LOCK_MID: dict[int | None, float] = {
    None: 1.0,   # prosta
    1:    0.75,  # łagodny — lekko odpuść
    2:    0.35,  # średni
    3:    0.05,  # ostry — prawie otwarty
}

# Blokada przy wejściu w zakręt (severity rośnie)
_LOCK_ENTERING: dict[int | None, float] = {
    None: 1.0,
    1:    0.65,
    2:    0.20,
    3:    0.00,
}


def compute_differential_lock(
    severity: int | None,
    prev_severity: int | None,
    throttle: float,
) -> float:
    """Blokada dyferencjału [0.0–1.0] na podstawie severity z classify_turn.

    severity / prev_severity:
      None — prosta
      1    — zakręt łagodny (~12°)
      2    — zakręt średni (~28°)
      3    — zakręt ostry (>28°)

    Logika:
      - prosta → pełna blokada (1.0)
      - wejście w zakręt (severity rośnie) → agresywne otwarcie
      - środek zakrętu (severity stałe) → blokada zależna od severity
      - wyjście z zakrętu (severity maleje) → narastaj z throttle
    """
    if severity is None:
        return 1.0

    prev_s = prev_severity if prev_severity is not None else 0
    curr_s = severity

    entering = curr_s > prev_s
    exiting  = curr_s < prev_s

    if entering:
        return _LOCK_ENTERING.get(severity, 0.0)

    if exiting:
        base = _LOCK_MID.get(severity, 0.5)
        return min(1.0, base + throttle * (1.0 - base))

    # środek zakrętu
    return _LOCK_MID.get(severity, 0.5)