from __future__ import annotations
import math

from hackarena3 import CenterlinePoint, Vec3

# minimalne angle_rad żeby uznać zakręt (brak zakrętu poniżej)
_TURN_THRESHOLD_RAD = 0.05   # ~3°
# angle_rad przy którym intensity = 1.0 (maksymalny zakręt)
_TURN_MAX_RAD = 1.2          # ~69°

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

def scan_turn_intensity(
    centerline: tuple[CenterlinePoint, ...],
    current_idx: int,
    scan_from_m: float,
    scan_to_m: float,
    sample_points: int = 5,
) -> tuple[float, bool | None]:  # <- tuple, nie float
    n = len(centerline)
    lap_length = centerline[-1].s_m
    current_s = centerline[current_idx].s_m

    from_s = current_s + scan_from_m
    to_s   = current_s + scan_to_m

    if from_s > lap_length:
        from_s -= lap_length
        to_s   -= lap_length

    max_intensity = 0.0
    dominant_cross = 0.0

    for i in range(n):
        s = centerline[i].s_m
        if not (from_s <= s <= to_s):
            continue

        end_idx = (i + sample_points) % n
        t_start = centerline[i].tangent
        t_end   = centerline[end_idx].tangent

        cross     = t_start.x * t_end.z - t_start.z * t_end.x
        dot       = t_start.x * t_end.x + t_start.z * t_end.z
        angle_rad = math.atan2(abs(cross), dot)

        if angle_rad < 0.05:
            continue

        intensity = min(1.0, (angle_rad - 0.05) / (1.2 - 0.05))

        if intensity > max_intensity:
            max_intensity = intensity
            dominant_cross = cross

    is_right = (dominant_cross < 0) if max_intensity > 0.0 else None

    return max_intensity, is_right  # <- zwróć tuple


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
