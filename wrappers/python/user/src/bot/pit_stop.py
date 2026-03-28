from __future__ import annotations

import math

from hackarena3 import CenterlinePoint, GearShift, PitstopLayout, Quaternion, Vec3

from steering import compute_steering, compute_steering_toward_direction

# Prędkość w alei serwisowej (km/h)
_PIT_LANE_SPEED_KMH = 60.0
# Dalsze zbliżanie do linii wejścia z toru
_APPROACH_ENTER_DIST_M = 35.0
_LOOKAHEAD_PIT_M = 10.0
# Punkt zatrzymania — środek odcinka „fix”
_STOP_NEAR_FIX_M = 4.0
_STOP_SPEED_KMH = 4.0


def find_closest_pit_stop_point(
    position: Vec3,
    pitstop: PitstopLayout,
) -> tuple[str, int, CenterlinePoint]:
    """Znajdź najbliższy punkt geometrii boksów (wejście / serwis / wyjście).

    Zwraca ``(segment, indeks_w_segmenie, punkt)``, gdzie ``segment`` to ``\"enter\"``, ``\"fix\"`` lub ``\"exit\"``.
    """
    best_seg = ""
    best_idx = 0
    best_point: CenterlinePoint | None = None
    best_dist = float("inf")

    for name, poly in (
        ("enter", pitstop.enter),
        ("fix", pitstop.fix),
        ("exit", pitstop.exit),
    ):
        for i, point in enumerate(poly):
            dx = position.x - point.position.x
            dy = position.y - point.position.y
            dz = position.z - point.position.z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist < best_dist:
                best_dist = dist
                best_seg = name
                best_idx = i
                best_point = point

    if best_point is None:
        raise ValueError("PitstopLayout has no pit geometry points (enter/fix/exit all empty)")

    return best_seg, best_idx, best_point


def _dist_xz(a: Vec3, b: Vec3) -> float:
    dx = a.x - b.x
    dz = a.z - b.z
    return math.hypot(dx, dz)


def _min_dist_to_poly(position: Vec3, poly: tuple[CenterlinePoint, ...]) -> float:
    if not poly:
        return float("inf")
    return min(_dist_xz(position, p.position) for p in poly)


def _closest_idx_on_poly(position: Vec3, poly: tuple[CenterlinePoint, ...]) -> int:
    best_i = 0
    best_d = float("inf")
    for i, p in enumerate(poly):
        d = _dist_xz(position, p.position)
        if d < best_d:
            best_d = d
            best_i = i
    return best_i


def _lookahead_on_poly(
    poly: tuple[CenterlinePoint, ...],
    current_idx: int,
    lookahead_m: float,
) -> CenterlinePoint:
    if not poly:
        raise ValueError("empty polyline")
    current_s = poly[current_idx].s_m
    target_s = current_s + lookahead_m
    seg_len = poly[-1].s_m
    if seg_len <= 0:
        return poly[min(current_idx, len(poly) - 1)]
    if target_s > seg_len:
        target_s = seg_len
    best = poly[current_idx]
    best_diff = float("inf")
    for point in poly:
        diff = abs(point.s_m - target_s)
        if diff < best_diff:
            best_diff = diff
            best = point
    return best


def _speed_control(speed_kmh: float, limit_kmh: float) -> tuple[float, float]:
    """Prosty P: gaz/hamulec żeby zmierzać do limit_kmh."""
    if speed_kmh <= limit_kmh + 1.0:
        if speed_kmh < limit_kmh - 2.0:
            return 0.35, 0.0
        return 0.2, 0.0
    over = speed_kmh - limit_kmh
    brake = min(0.65, 0.15 + over * 0.04)
    return 0.0, brake


class PitStopController:
    """Planowanie pit stopu na okrążeniu: dojazd do wejścia, ograniczenie prędkości, dojazd do fix, postój."""

    def __init__(self) -> None:
        self._target_lap: int | None = None
        self._sequence_active = False
        self._completed = False

    def request_pit_stop(self, lap: int) -> None:
        """Zaplanuj pit stop na danym numerze okrążenia (>= 1). Wywołaj z logiki bota."""
        self._target_lap = lap
        self._sequence_active = False
        self._completed = False

    def cancel_pit_stop(self) -> None:
        """Anuluj zaplanowany pit."""
        self._target_lap = None
        self._sequence_active = False
        self._completed = False

    def is_sequence_active(self) -> bool:
        return self._sequence_active and not self._completed

    def _lap_allows_start(self, current_lap: int | None) -> bool:
        if self._target_lap is None:
            return False
        # Brak telemetrii okrążenia — od razu można włączyć sekwencję (testy / własny licznik później)
        if current_lap is None:
            return True
        return current_lap >= self._target_lap

    def step(
        self,
        position: Vec3,
        orientation: Quaternion,
        speed_kmh: float,
        pitstop: PitstopLayout,
        tick: int,
        current_lap: int | None = None,
    ) -> tuple[float, float, float, GearShift] | None:
        """Zwraca (gaz, hamulec, skręt, bieg) podczas pit stopu, albo ``None`` gdy sekwencja nieaktywna.

        ``current_lap`` — bieżące okrążenie z telemetrii; jeśli ``None``, sekwencja startuje
        po ``request_pit_stop`` bez czekania na okrążenie.
        """
        if not self._lap_allows_start(current_lap):
            return None

        if not pitstop.enter:
            return None

        self._sequence_active = True

        enter = pitstop.enter
        fix = pitstop.fix
        seg, idx, _cp = find_closest_pit_stop_point(position, pitstop)

        dist_to_enter_line = _min_dist_to_poly(position, enter)
        # 1) Daleko od linii wejścia — celuj w pierwszy punkt wejścia do boksów
        if dist_to_enter_line > _APPROACH_ENTER_DIST_M:
            t = enter[0].position
            dx = t.x - position.x
            dz = t.z - position.z
            steer = compute_steering_toward_direction(orientation, dx, dz, tick)
            thr, brk = _speed_control(speed_kmh, min(120.0, _PIT_LANE_SPEED_KMH + 50.0))
            return thr, brk, steer, GearShift.NONE

        # 2) Strefa fix — podążaj ścieżką „fix”, zatrzymaj się przy stanowisku
        if seg == "fix" and fix:
            fix_idx = _closest_idx_on_poly(position, fix)
            la = _lookahead_on_poly(fix, fix_idx, _LOOKAHEAD_PIT_M)
            steer = compute_steering(orientation, la.tangent, tick)

            stop_pt = fix[len(fix) // 2]
            d_fix = _dist_xz(position, stop_pt.position)

            if d_fix <= _STOP_NEAR_FIX_M and speed_kmh > _STOP_SPEED_KMH:
                return 0.0, min(1.0, 0.5 + (_STOP_NEAR_FIX_M - d_fix) * 0.15), steer, GearShift.NONE

            if d_fix <= _STOP_NEAR_FIX_M * 0.65 and speed_kmh <= _STOP_SPEED_KMH:
                self._completed = True
                self._sequence_active = False
                self._target_lap = None
                return 0.0, 1.0, 0.0, GearShift.NONE

            thr, brk = _speed_control(speed_kmh, _PIT_LANE_SPEED_KMH * 0.75)
            return thr, brk, steer, GearShift.NONE

        # 3) Odcinek wejścia (pit lane enter) — tangentą, ograniczona prędkość
        if seg == "enter":
            cidx = _closest_idx_on_poly(position, enter)
            la = _lookahead_on_poly(enter, cidx, _LOOKAHEAD_PIT_M)
            steer = compute_steering(orientation, la.tangent, tick)
            thr, brk = _speed_control(speed_kmh, _PIT_LANE_SPEED_KMH)
            return thr, brk, steer, GearShift.NONE

        # Domyślnie: np. exit — na razie delikatnie trzymaj się łamanej
        poly_map = {"enter": enter, "fix": fix, "exit": pitstop.exit}
        poly = poly_map.get(seg) or enter
        if not poly:
            return 0.0, 0.3, 0.0, GearShift.NONE
        cidx = _closest_idx_on_poly(position, poly)
        la = _lookahead_on_poly(poly, cidx, _LOOKAHEAD_PIT_M)
        steer = compute_steering(orientation, la.tangent, tick)
        thr, brk = _speed_control(speed_kmh, _PIT_LANE_SPEED_KMH)
        return thr, brk, steer, GearShift.NONE
