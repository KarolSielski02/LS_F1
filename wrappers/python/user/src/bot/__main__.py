from __future__ import annotations
from debug import init_debug_csv, write_debug_row
import math

from dotenv import load_dotenv
from hackarena3 import BotContext, DriveGear, GearShift, RaceSnapshot, TireType, run_bot, CenterlinePoint, Vec3

load_dotenv()

speedTypes = {
    "fastest": 150.0,
    "fast": 80.0,
    "normal": 35.0,
    "slow": 10.0
}

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

    # znajdź indeks punktu startowego
    start_idx = 0
    best_diff = float("inf")
    for i, point in enumerate(centerline):
        diff = abs(point.s_m - target_s)
        if diff < best_diff:
            best_diff = diff
            start_idx = i

    # zbierz sample_points kolejnych punktów z zawijaniem
    n = len(centerline)
    indices = [(start_idx + i) % n for i in range(sample_points + 1)]
    points = [centerline[i] for i in indices]

    t_start = points[0].tangent
    t_end = points[-1].tangent

    # iloczyn wektorowy tangentów → jak bardzo i w którą stronę skręcił kierunek jazdy
    # cross > 0 = skręt w lewo, cross < 0 = skręt w prawo
    cross = t_start.x * t_end.z - t_start.z * t_end.x

    # iloczyn skalarny → jak bardzo tangenty są zgodne (1.0 = identyczne, -1.0 = przeciwne)
    dot   = t_start.x * t_end.x + t_start.z * t_end.z

    # kąt między tangentami w radianach
    angle_rad = math.atan2(abs(cross), dot)
    if tick % 50 == 0:
        print('cross', cross)
        print('dot', dot)
        print('angle_rad ', angle_rad)

    if angle_rad < 0.05:  # ~3°
        return False, None, None

    is_right = cross < 0

    if angle_rad < 0.2:   # ~11°
        severity = 1
    elif angle_rad < 0.5: # ~29°
        severity = 2
    else:
        severity = 3

    return True, is_right, severity


def compute_steering(
    orientation: Quaternion,
    target_tangent: Vec3,
    tick: int
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


def compute_throttle_brake(
    speed_kmh: float,
    
) -> tuple[float, float]:
    """Dobierz gaz i hamulec na podstawie prędkości, krzywizny i nachylenia."""
    targetSpeed = 30.0

    if targetSpeed > speed_kmh:
        return 0.3, 0.0
    else:
        return 0.01, 0.0


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
        self.prevIndex = 0

    def on_tick(self, snapshot: RaceSnapshot, ctx: BotContext) -> None:
        self._tick += 1
        car = snapshot.car

        if self._tick <= self.WARMUP_TICKS:
            return

        centerline = ctx.track.centerline
        centerlineLength = len(centerline)
        current_idx, current_point = find_closest_centerline_point(car.position, centerline)
        
        steering = compute_steering(
            car.orientation,
            get_lookahead_point(centerline, current_idx, self.LOOKAHEAD_M).tangent,
            self._tick
        )

        throttle, brake = compute_throttle_brake(
            car.speed_kmh,
            # get_lookahead_point(centerline, current_idx, self.BRAKE_LOOKAHEAD_M).curvature_1pm,
            # current_point.grade_rad,
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

        # if snapshot.tick % 10 == 0:
        #     write_debug_row(
        #         writer=self._csv_writer,
        #         snapshot=snapshot,
        #         ctx=ctx,
        #         current_idx=current_idx,
        #         current_point=current_point,
        #         lookahead_m=lookahead_m,
        #         brake_lookahead_m=brake_lookahead_m,
        #         steering=steering,
        #         throttle=throttle,
        #         brake=brake,
        #         max_slip=max_slip,
        #         min_wear=min_wear,
        #     )
            
        if self._tick % 50 == 0:
            print('orientation: ', car.orientation)
            print('get_lookahead_point.tangent: ',get_lookahead_point(centerline, current_idx, self.LOOKAHEAD_M).tangent),
        #     print('speed: ')
        #     print(throttle)
        #     print('steer: ')
        #     print(steering)
        #     print('amount of points')
        #     print(centerlineLength)

        ctx.set_controls(
            throttle=throttle,
            brake=brake,
            steer=steering,
            gear_shift=compute_gear_shift(int(car.gear), car.engine_rpm),
        )


if __name__ == "__main__":
    raise SystemExit(run_bot(BasicBot()))