from __future__ import annotations
from debug import init_debug_csv, write_debug_row
import math

from dotenv import load_dotenv
from hackarena3 import BotContext, DriveGear, GearShift, RaceSnapshot, TireType, run_bot, CenterlinePoint, Vec3

load_dotenv()


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
    target_s = centerline[current_idx].s_m + lookahead_m
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


def compute_steering(
    position: Vec3,
    target: CenterlinePoint,
    right: Vec3,
) -> float:
    """Oblicz skręt kierownicy w kierunku punktu docelowego."""
    dx = target.position.x - position.x
    dy = target.position.y - position.y
    dz = target.position.z - position.z

    dot = dx * right.x + dy * right.y + dz * right.z
    return max(-1.0, min(1.0, dot / 20.0))


def compute_throttle_brake(
    speed_mps: float,
    curvature: float,
    grade_rad: float,
) -> tuple[float, float]:
    """Dobierz gaz i hamulec na podstawie prędkości, krzywizny i nachylenia."""
    abs_curvature = abs(curvature)

    if abs_curvature < 0.001:
        target_speed_mps = 80.0
    else:
        target_speed_mps = math.sqrt(30.0 / abs_curvature)
        target_speed_mps = max(15.0, min(80.0, target_speed_mps))

    speed_error = target_speed_mps - speed_mps + math.sin(grade_rad) * 10.0

    if speed_error > 0:
        return max(0.0, min(1.0, speed_error / 20.0)), 0.0
    else:
        return 0.0, min(1.0, (-speed_error) / 20.0)


def compute_gear_shift(gear: int, engine_rpm: float) -> GearShift:
    """Prosta automatyczna skrzynia biegów na podstawie RPM."""
    if engine_rpm > 6500 and gear < 8:
        return GearShift.UPSHIFT
    if engine_rpm < 2500 and gear > 1:
        return GearShift.DOWNSHIFT
    return GearShift.NONE


class BasicBot:
    WARMUP_TICKS = 50
    LOOKAHEAD_M = 15.0
    BRAKE_LOOKAHEAD_M = 45.0
    TIRE_WEAR_THRESHOLD = 0.20
    SLIP_THRESHOLD = 1.0

    def __init__(self) -> None:
        self._tick = 0
        self._csv_writer, self._csv_file = init_debug_csv()

    def on_tick(self, snapshot: RaceSnapshot, ctx: BotContext) -> None:
        self._tick += 1
        car = snapshot.car

        if self._tick <= self.WARMUP_TICKS:
            return

        if car.gear in (DriveGear.REVERSE, DriveGear.NEUTRAL):
            ctx.set_controls(
                throttle=0.0,
                brake=0.3,
                steer=0.0,
                gear_shift=GearShift.UPSHIFT,
            )
            return

        centerline = ctx.track.centerline
        current_idx, current_point = find_closest_centerline_point(car.position, centerline)

        steering = compute_steering(
            car.position,
            get_lookahead_point(centerline, current_idx, self.LOOKAHEAD_M),
            current_point.right,
        )

        throttle, brake = compute_throttle_brake(
            car.speed_mps,
            get_lookahead_point(centerline, current_idx, self.BRAKE_LOOKAHEAD_M).curvature_1pm,
            current_point.grade_rad,
        )

        max_slip = max(
            car.tire_slip.front_left,
            car.tire_slip.front_right,
            car.tire_slip.rear_left,
            car.tire_slip.rear_right,
        )
        if max_slip > self.SLIP_THRESHOLD:
            throttle = 0.0
            brake = max(brake, 0.1)

        min_wear = min(
            car.tire_wear.front_left,
            car.tire_wear.front_right,
            car.tire_wear.rear_left,
            car.tire_wear.rear_right,
        )
        if min_wear < self.TIRE_WEAR_THRESHOLD and not car.pit_request_active:
            ctx.set_next_pit_tire_type(TireType.SOFT)
            ctx.request_emergency_pitstop()

        if snapshot.tick % 10 == 0:
            write_debug_row(
            writer=self._csv_writer,
            snapshot=snapshot,
            ctx=ctx,
            current_idx=current_idx,
            current_point=current_point,
            lookahead_m=lookahead_m,
            brake_lookahead_m=brake_lookahead_m,
            steering=steering,
            throttle=throttle,
            brake=brake,
            max_slip=max_slip,
            min_wear=min_wear,
        )

        ctx.set_controls(
            throttle=throttle,
            brake=brake,
            steer=steering,
            gear_shift=compute_gear_shift(int(car.gear), car.engine_rpm),
        )


if __name__ == "__main__":
    raise SystemExit(run_bot(BasicBot()))