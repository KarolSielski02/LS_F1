def compute_differential_lock(
    intensity: float,
    prev_intensity: float,
    throttle: float,
) -> float:
    # prosta lub bardzo łagodny zakręt
    if intensity < 0.1:
        return 1.0

    entering = intensity > prev_intensity
    exiting  = intensity < prev_intensity

    if entering:
        return max(0.0, 1.0 - intensity * 2.0)   # 0.5→0.0 przy intensity 0.5+

    if exiting:
        return 0.5 + throttle * 0.5

    # środek zakrętu
    return max(0.05, 1.0 - intensity * 1.5)