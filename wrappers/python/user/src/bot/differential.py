def compute_differential_lock(
    is_turn: bool,
    severity: int | None,
    prev_is_turn: bool,
    prev_severity: int | None,
    throttle: float,
) -> float:
    """Dyferencjał oparty na fazie zakrętu i gazie.

    severity 1 (sweeper)  → traktuj jak prostą, pełna blokada
    severity 2–3          → pełna strategia wejście/środek/wyjście
    """
    current  = severity or 0
    previous = prev_severity or 0

    # prosta lub sweeper — pełna blokada
    if not is_turn or current <= 1:
        return 1.0

    # od tej pory severity 2 lub 3
    entering = current > previous or (is_turn and not prev_is_turn)
    exiting  = current < previous and prev_is_turn

    if entering:
        # severity 2 → 0.2, severity 3 → 0.0
        return max(0.0, 0.4 - current * 0.2)

    if exiting:
        return 0.5 + throttle * 0.5

    # środek zakrętu
    # severity 2 → 0.2, severity 3 → 0.1
    return max(0.1, 0.4 - current * 0.15)