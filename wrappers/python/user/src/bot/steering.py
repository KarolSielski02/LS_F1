from __future__ import annotations
import math

from hackarena3 import CenterlinePoint, Vec3


def find_closest_centerline_point(
    position: Vec3,
    centerline: tuple[CenterlinePoint, ...],
) -> tuple[int, CenterlinePoint]:
    """Znajdź punkt na osi centralnej najbliższy aktualnej pozycji samochodu."""
    best_idx = 0
    best_dist = float("inf")

    for i, point in enumerate(centerline):
        dx = position.x - point.position.x
        dy = position.y - point.position.y
        dz = position.z - point.position.z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist < best_dist:
            best_dist = dist
            best_idx = i

    return best_idx, centerline[best_idx]


def get_lookahead_point(
    centerline: tuple[CenterlinePoint, ...],
    current_idx: int,
    lookahead_m: float,
) -> CenterlinePoint:
    """Zwróć punkt na torze oddalony o lookahead_m od aktualnej pozycji."""
    current_s = centerline[current_idx].s_m
    target_s = current_s + lookahead_m
    lap_length = centerline[-1].s_m

    if target_s > lap_length:
        target_s -= lap_length

    best = centerline[current_idx]
    best_diff = float("inf")

    for point in centerline:
        diff = abs(point.s_m - target_s)
        if diff < best_diff:
            best_diff = diff
            best = point

    return best


def classify_turn(
    centerline: tuple[CenterlinePoint, ...],
    current_idx: int,
    lookahead_m: float,
    tick: int,
    sample_points: int = 5,
) -> tuple[bool, bool | None, int | None]:
    """Sprawdź czy za lookahead_m zaczyna się zakręt porównując tangenty kolejnych punktów.

    Zwraca (is_turn, is_right, severity) gdzie:
      is_turn    - czy w ogóle jest zakręt
      is_right   - True = zakręt w prawo, False = w lewo, None = prosta
      severity   - 1 łagodny, 2 średni, 3 ostry, None = prosta
    """
    target_s = centerline[current_idx].s_m + lookahead_m
    lap_length = centerline[-1].s_m
    if target_s > lap_length:
        target_s -= lap_length

    start_idx = 0
    best_diff = float("inf")
    for i, point in enumerate(centerline):
        diff = abs(point.s_m - target_s)
        if diff < best_diff:
            best_diff = diff
            start_idx = i

    n = len(centerline)
    indices = [(start_idx + i) % n for i in range(sample_points + 1)]
    points = [centerline[i] for i in indices]

    t_start = points[0].tangent
    t_end = points[-1].tangent

    # cross > 0 = skręt w lewo, cross < 0 = skręt w prawo
    cross = t_start.x * t_end.z - t_start.z * t_end.x
    dot   = t_start.x * t_end.x + t_start.z * t_end.z

    angle_rad = math.atan2(abs(cross), dot)
    if tick % 50 == 0:
        print('cross', cross)
        print('dot', dot)
        print('angle_rad ', angle_rad)

    if angle_rad < 0.09:  # ~5°
        return False, None, None

    is_right = cross < 0

    if angle_rad < 0.13:   # ~12°
        severity = 1
    elif angle_rad < 0.25: # ~28°
        severity = 2
    elif angle_rad < 0.37:
        severity = 3
    else:
        severity = 4

    return True, is_right, severity


def compute_steering(
    orientation,
    target_tangent: Vec3,
    tick: int,
) -> float:
    qx, qy, qz, qw = orientation.x, orientation.y, orientation.z, orientation.w

    # przód auta na osi Z — obracamy wektor (0, 0, 1) kwaternionem
    forward_x = 2.0 * (qx * qz + qy * qw)
    forward_z = 1.0 - 2.0 * (qx * qx + qy * qy)

    cross = forward_x * target_tangent.z - forward_z * target_tangent.x
    dot   = forward_x * target_tangent.x + forward_z * target_tangent.z
    angle = math.atan2(cross, dot)

    if tick % 50 == 0:
        print(f"forward: x={forward_x:.3f} z={forward_z:.3f} | target: x={target_tangent.x:.3f} z={target_tangent.z:.3f}")
        print(f"cross={cross:.3f} dot={dot:.3f} angle_deg={math.degrees(angle):.1f}")

    steering = angle / (math.pi / 2)

    if steering<-0.19:
        steering -= 0.07
    elif steering>0.19:
        steering += 0.07

    return max(-1.0, min(1.0, steering))


def compute_steering_toward_direction(
    orientation,
    dir_x: float,
    dir_z: float,
    tick: int,
) -> float:
    """Align car forward with (dir_x, dir_z) in the XZ plane."""
    ln = math.hypot(dir_x, dir_z)
    if ln < 1e-6:
        return 0.0
    tx, tz = dir_x / ln, dir_z / ln
    return compute_steering(orientation, Vec3(x=tx, y=0.0, z=tz), tick)
