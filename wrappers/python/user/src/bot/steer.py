from __future__ import annotations

import math
from dataclasses import dataclass

from hackarena3 import CenterlinePoint, Vec3

# ── Steering ─────────────────────────────────────────────────────────────────

# How far ahead along the tangent direction to place the steering target [m]
# Larger = smoother but slower to react; smaller = sharper but twitchy
TANGENT_STEP_M: float = 10.0

# Cross-track error gain — how aggressively to pull back to centerline
# 0.0 = stay parallel to tangent regardless of offset
# 1.0 = strong pull toward center on every tick
CTE_GAIN: float = 0.8

# Differential lock on a straight (steer ≈ 0) — equal torque, no spin
DIFF_LOCK_STRAIGHT: float = 1.0

# Differential lock at full lock (steer ≈ ±1) — open, wheels rotate freely
DIFF_LOCK_CORNER: float = 0.0


def find_closest_centerline_point(
    position: Vec3,
    centerline: tuple[CenterlinePoint, ...],
) -> tuple[int, CenterlinePoint]:
    """Find the centerline point closest to the car's current position."""
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


def compute_steering(
    position: Vec3,
    current_point: CenterlinePoint,
) -> float:
    """Steer the car to follow the tangent of the closest centerline point.

    Every tick:
    1. Compute cross-track error (CTE) — how far left/right the car is from
       the centerline, along the `right` unit vector of the current point.
    2. Build a target point = current_point.position
                             + TANGENT_STEP_M * tangent   (step forward along track)
                             - CTE_GAIN * cte * right     (pull back to center)
    3. Compute the angle from car position to target, projected onto `right`.

    This naturally:
    - Keeps the car aligned with the tangent (heading correction)
    - Keeps the car centered on the track (CTE correction)
    - No quaternion math needed — geometry only
    """
    cp = current_point
    tangent = cp.tangent
    right = cp.right

    # Signed lateral offset from centerline (+ve = car is to the right)
    off_x = position.x - cp.position.x
    off_z = position.z - cp.position.z
    cte = off_x * right.x + off_z * right.z

    # Target: step along tangent from centerline, with centering correction
    lateral_correction = -CTE_GAIN * cte
    target_x = cp.position.x + TANGENT_STEP_M * tangent.x + lateral_correction * right.x
    target_z = cp.position.z + TANGENT_STEP_M * tangent.z + lateral_correction * right.z

    # Direction from car to target
    dx = target_x - position.x
    dz = target_z - position.z
    length = math.hypot(dx, dz)
    if length < 1e-6:
        return 0.0

    dir_x = dx / length
    dir_z = dz / length

    # Project onto right vector → steering value [-1, 1]
    steer = dir_x * right.x + dir_z * right.z
    return max(-1.0, min(1.0, steer))


def compute_differential_lock(steer: float) -> float:
    """Lock differential on straights, open on corners.

    Straight (steer ≈ 0) → lock 1.0: equal torque both rear wheels, no spin.
    Full lock (steer ≈ ±1) → lock 0.0: wheels free for cornering.
    """
    return max(DIFF_LOCK_CORNER, DIFF_LOCK_STRAIGHT * (1.0 - abs(steer)))


# ── Corner prediction ─────────────────────────────────────────────────────────

# How far ahead to scan for corners [m]
SCAN_AHEAD_M: float = 200.0

# Curvature [1/m] above which a segment is treated as a corner (radius < 200m)
CURVATURE_THRESHOLD: float = 0.005

# Assumed max lateral acceleration [m/s²] — lower = more conservative corner speed
LATERAL_ACCEL_MPS2: float = 3.5

# Assumed braking deceleration [m/s²]
DECELERATION_MPS2: float = 12.0

# Safety margin on top of calculated braking distance [m]
BRAKE_MARGIN_M: float = 8.0

# Distance at which car is considered to be inside the corner — braking stops
MIN_BRAKE_DIST_M: float = 8.0

# Extra coast distance before the braking zone (throttle off, no brake)
COAST_EXTRA_M: float = 15.0


@dataclass
class CornerInfo:
    """Result of a corner scan — drives both throttle and brake decisions."""
    detected: bool           # True if corner found within SCAN_AHEAD_M
    distance_m: float        # Distance to corner entry [m]
    max_curvature: float     # Peak curvature [1/m]
    target_speed_kmh: float  # Safe entry speed [km/h]
    brake_level: float       # Brake pressure [0.0–1.0]
    throttle_allowed: bool   # False = lift off gas (coast / brake / in-corner)


def _safe_corner_speed(max_curvature: float) -> float:
    """Maximum safe entry speed [km/h]: v = sqrt(lateral_accel / curvature)."""
    if max_curvature < 1e-6:
        return 999.0
    return math.sqrt(LATERAL_ACCEL_MPS2 / max_curvature) * 3.6


def _progressive_brake_level(
    speed_kmh: float,
    target_kmh: float,
    dist_m: float,
) -> float:
    """Physically-correct brake pressure for this exact moment.

    Required deceleration: a = (v² - v_target²) / (2 * dist)
    Normalised by max deceleration to get [0, 1] brake level.
    """
    v_cur = speed_kmh / 3.6
    v_tgt = target_kmh / 3.6
    if v_cur <= v_tgt:
        return 0.0
    a = (v_cur ** 2 - v_tgt ** 2) / (2.0 * max(1.0, dist_m))
    return min(1.0, a / DECELERATION_MPS2)


def predict_corner(
    centerline: tuple[CenterlinePoint, ...],
    current_idx: int,
    speed_kmh: float,
) -> CornerInfo:
    """Scan ahead and classify driving phase:

    STRAIGHT  → throttle ON,  brake off
    COAST     → throttle OFF, brake off   (entered coast window)
    BRAKE     → throttle OFF, progressive brake
    IN CORNER → throttle OFF, brake off   (< MIN_BRAKE_DIST_M, steer only)
    """
    n = len(centerline)
    current_s = centerline[current_idx].s_m
    lap_length = centerline[-1].s_m

    corner_start_dist: float | None = None
    max_curvature: float = 0.0
    in_corner = False
    straight_gap = 0

    for offset in range(1, n):
        idx = (current_idx + offset) % n
        point = centerline[idx]

        dist = point.s_m - current_s
        if dist < 0:
            dist += lap_length
        if dist > SCAN_AHEAD_M:
            break

        curvature = abs(point.curvature_1pm)

        if curvature >= CURVATURE_THRESHOLD:
            if not in_corner:
                corner_start_dist = dist
                in_corner = True
            max_curvature = max(max_curvature, curvature)
            straight_gap = 0
        elif in_corner:
            straight_gap += 1
            if straight_gap > 3:
                break

    # No corner ahead
    if corner_start_dist is None:
        return CornerInfo(
            detected=False, distance_m=SCAN_AHEAD_M, max_curvature=0.0,
            target_speed_kmh=999.0, brake_level=0.0, throttle_allowed=True,
        )

    target_kmh = _safe_corner_speed(max_curvature)

    # Phase: IN CORNER
    if corner_start_dist < MIN_BRAKE_DIST_M:
        return CornerInfo(
            detected=True, distance_m=corner_start_dist, max_curvature=max_curvature,
            target_speed_kmh=target_kmh, brake_level=0.0, throttle_allowed=False,
        )

    braking_dist = (
        max(0.0, (speed_kmh / 3.6) ** 2 - (target_kmh / 3.6) ** 2)
        / (2.0 * DECELERATION_MPS2)
    ) + BRAKE_MARGIN_M

    # Phase: BRAKE
    if corner_start_dist <= braking_dist:
        return CornerInfo(
            detected=True, distance_m=corner_start_dist, max_curvature=max_curvature,
            target_speed_kmh=target_kmh,
            brake_level=_progressive_brake_level(speed_kmh, target_kmh, corner_start_dist),
            throttle_allowed=False,
        )

    # Phase: COAST (once inside window, throttle stays off — no flip-flop)
    if corner_start_dist <= braking_dist + COAST_EXTRA_M:
        return CornerInfo(
            detected=True, distance_m=corner_start_dist, max_curvature=max_curvature,
            target_speed_kmh=target_kmh, brake_level=0.0, throttle_allowed=False,
        )

    # Phase: STRAIGHT
    return CornerInfo(
        detected=True, distance_m=corner_start_dist, max_curvature=max_curvature,
        target_speed_kmh=target_kmh, brake_level=0.0, throttle_allowed=True,
    )
