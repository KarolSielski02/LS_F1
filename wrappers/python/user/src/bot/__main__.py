from __future__ import annotations
from debug import init_debug_csv, write_debug_row
from gear import compute_gear_shift
from braking import BrakeController
from throttle import ThrottleController
from benchmark import BrakeBenchmark
from steer import (
    find_closest_centerline_point,
    compute_steering, compute_differential_lock,
    predict_corner,
)

from dotenv import load_dotenv
from hackarena3 import BotContext, GearShift, RaceSnapshot, TireType, run_bot

load_dotenv()


class BasicBot:
    WARMUP_TICKS = 50
    TIRE_WEAR_THRESHOLD = 0.20
    SLIP_THRESHOLD = 0.5

    def __init__(self) -> None:
        self._tick = 0
        self._csv_writer, self._csv_file = init_debug_csv()
        self.prevIndex = 0
        self._last_shift_ms = -1000
        self._brakes = BrakeController()
        self._throttle = ThrottleController()
        self._benchmark = BrakeBenchmark(delay_ms=10_000, brake_level=0.8, duration_ms=2_000)

    def on_tick(self, snapshot: RaceSnapshot, ctx: BotContext) -> None:
        self._tick += 1
        car = snapshot.car

        if self._tick <= self.WARMUP_TICKS:
            return

        centerline = ctx.track.centerline
        centerlineLength = len(centerline)
        current_idx, current_point = find_closest_centerline_point(car.position, centerline)
        
        steering = compute_steering(car.position, current_point)

        max_slip = max(
            car.tire_slip.front_left,
            car.tire_slip.front_right,
            car.tire_slip.rear_left,
            car.tire_slip.rear_right,
        )

        avg_tire_temp = (
            car.tire_temperature_celsius.front_left_celsius
            + car.tire_temperature_celsius.front_right_celsius
            + car.tire_temperature_celsius.rear_left_celsius
            + car.tire_temperature_celsius.rear_right_celsius
        ) / 4.0

        # Corner prediction — phase-based throttle and brake control
        corner = predict_corner(centerline, current_idx, car.speed_kmh)
        if corner.brake_level > 0.0:
            self._brakes.request(corner.brake_level)
        else:
            self._brakes.release()

        self._benchmark.update(self._brakes, snapshot.server_time_ms)
        brake, _ = self._brakes.tick()

        if not corner.throttle_allowed:
            # Coast / brake / in-corner phase — minimal throttle to keep engine loaded
            throttle = 0.05
            self._throttle.reset(0.05)
        else:
            throttle = self._throttle.tick(
                steer=steering,
                max_slip=max_slip,
                slip_threshold=self.SLIP_THRESHOLD,
                avg_tire_temp=avg_tire_temp,
                is_braking=self._brakes.is_braking,
            )

        differential_lock = compute_differential_lock(steering)

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
                lookahead_m=10.0,
                brake_lookahead_m=45.0,
                steering=steering,
                throttle=throttle,
                brake=brake,
                max_slip=max_slip,
                min_wear=min_wear,
            )
            
        if self._tick % 50 == 0:
            print('speed: ')
            print(throttle)
            print('steer: ')
            print(steering)
            print('amount of points')
            print(centerlineLength)

        gear_shift = compute_gear_shift(
            int(car.gear),
            car.speed_kmh,
            car.engine_rpm,
            throttle,
            brake,
            snapshot.server_time_ms,
            self._last_shift_ms,
        )
        if gear_shift != GearShift.NONE:
            self._last_shift_ms = snapshot.server_time_ms

        ctx.set_controls(
            throttle=throttle,
            brake=brake,
            steer=steering,
            gear_shift=gear_shift,
            differential_lock=differential_lock,
        )


if __name__ == "__main__":
    raise SystemExit(run_bot(BasicBot()))