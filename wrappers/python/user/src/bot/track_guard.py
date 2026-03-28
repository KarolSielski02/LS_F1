from __future__ import annotations
import math

from hackarena3 import CenterlinePoint, Vec3

# Od jakiej części połowy szerokości toru zaczyna się delikatne „pchnięcie” kierownicą (0 = oś, 1 = krawędź)
NUDGE_THRESHOLD = 0.6   # np. po 60% połowy szerokości
HARD_THRESHOLD  = 1.0   # pełny poza torem dopiero przy krawędzi / poza

# Maks. dodatkowy skręt (mieszany z normalnym) przy nudge
MAX_NUDGE_STEER = 0.4

# Kąt między nosem a tangenty toru powyżej którego uznajemy „złą stronę” (~50°)
WRONG_WAY_ANGLE_RAD = math.radians(50)


class TrackStatus:
    def __init__(
        self,
        lateral_m: float,       # ze znakiem: + = na lewo od osi, − = na prawo
        left_width_m: float,
        right_width_m: float,
        wrong_way: bool,
        heading_error_rad: float,
    ) -> None:
        self.lateral_m       = lateral_m
        self.left_width_m    = left_width_m
        self.right_width_m   = right_width_m
        self.wrong_way       = wrong_way
        self.heading_error_rad = heading_error_rad

    @property
    def _half_width(self) -> float:
        return self.left_width_m if self.lateral_m >= 0 else self.right_width_m

    @property
    def edge_fraction(self) -> float:
        """0.0 = na osi, 1.0 = przy krawędzi pasa, >1.0 = poza torem."""
        hw = self._half_width
        return abs(self.lateral_m) / hw if hw > 0 else 0.0

    @property
    def off_track(self) -> bool:
        return self.edge_fraction >= HARD_THRESHOLD

    @property
    def needs_nudge(self) -> bool:
        return self.edge_fraction >= NUDGE_THRESHOLD or self.wrong_way


class TrackGuard:
    def __init__(self) -> None:
        self._recovery_ticks = 0

    def check(
        self,
        position: Vec3,
        orientation,
        centerline: tuple[CenterlinePoint, ...],
        current_idx: int,
    ) -> TrackStatus:
        cp = centerline[current_idx]
        tx, tz = cp.tangent.x, cp.tangent.z

        # Odległość boczna względem osi: rzut wektora auto–punkt na lewą normalną do tangenty
        # Lewa normalna do (tx, tz) to (−tz, tx)
        dx = position.x - cp.position.x
        dz = position.z - cp.position.z
        lateral_m = dx * (-tz) + dz * tx   # + = lewo od osi, − = prawo

        # Wektor „przód” z kwaternionu
        qx, qy, qz, qw = orientation.x, orientation.y, orientation.z, orientation.w
        forward_x = 2.0 * (qx * qz + qy * qw)
        forward_z = 1.0 - 2.0 * (qx * qx + qy * qy)

        cross = forward_x * tz - forward_z * tx
        dot   = forward_x * tx + forward_z * tz
        heading_error_rad = math.atan2(abs(cross), dot)
        wrong_way = heading_error_rad > WRONG_WAY_ANGLE_RAD

        return TrackStatus(
            lateral_m,
            cp.left_width_m,
            cp.right_width_m,
            wrong_way,
            heading_error_rad,
        )

    def apply_recovery(
        self,
        status: TrackStatus,
        normal_throttle: float,
        normal_brake: float,
        normal_steer: float,
        centerline: tuple[CenterlinePoint, ...],
        current_idx: int,
        orientation,
    ) -> tuple[float, float, float]:
        """Delikatne dociąganie skrętu przy szerokim jeździe. Całkowity wyjazd = detrack_recovery."""
        if status.off_track:
            self._recovery_ticks = 0
            return normal_throttle, normal_brake, normal_steer

        # Złą stronę obsługuje detrack_recovery (request_back_to_track); tu nie gazujemy na zero
        if status.wrong_way:
            self._recovery_ticks = 0
            return normal_throttle, normal_brake, normal_steer

        if not status.needs_nudge:
            self._recovery_ticks = 0
            return normal_throttle, normal_brake, normal_steer

        self._recovery_ticks += 1

        # Siła nudge: 0 przy NUDGE_THRESHOLD, 1.0 przy HARD_THRESHOLD i dalej
        nudge_range = HARD_THRESHOLD - NUDGE_THRESHOLD
        strength = min(1.0, (status.edge_fraction - NUDGE_THRESHOLD) / nudge_range)

        # Kierunek do środka: przeciwnie do bocznego offsetu
        nudge_direction = -1.0 if status.lateral_m > 0 else 1.0   # lewo na torze = skręt w prawo itd.
        nudge_steer = nudge_direction * MAX_NUDGE_STEER * strength

        # Łagodne mieszanie ze skrętem bazowym, gaz/ham bez zmian
        blended_steer = max(-1.0, min(1.0, normal_steer + nudge_steer))
        if self._recovery_ticks % 30 == 0:
            print(f"[TrackGuard] nudge  edge={status.edge_fraction:.2f} "
                  f"lateral={status.lateral_m:.1f}m steer_delta={nudge_steer:+.2f}")
        return normal_throttle, normal_brake, blended_steer
