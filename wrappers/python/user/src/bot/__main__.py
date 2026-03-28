from __future__ import annotations
from debug import init_debug_csv, write_debug_row

from dotenv import load_dotenv
from hackarena3 import BotContext, DriveGear, RaceSnapshot, TireType, run_bot, CenterlinePoint, Vec3
from steering import find_closest_centerline_point, get_lookahead_point, classify_turn, compute_steering
from gear import GearController
from throttle import ThrottleController
from track_guard import TrackGuard
from detrack_recovery import DetrackRecovery

load_dotenv()

speedTypes = {
    "fastest": 150.0,
    "fast": 80.0,
    "normal": 35.0,
    "slow": 10.0
}



class BasicBot:
    WARMUP_TICKS = 50
    LOOKAHEAD_M = 15.0
    TIRE_WEAR_THRESHOLD = 0.20
    SLIP_THRESHOLD = 1.0

    # Brake lookahead scales with speed: ~1.8 s of travel → 100 m at 200 km/h
    BRAKE_LOOKAHEAD_REACTION_S = 1.8
    BRAKE_LOOKAHEAD_MIN_M = 40.0
    BRAKE_LOOKAHEAD_MAX_M = 110.0

    @staticmethod
    def brake_lookahead_m(speed_kmh: float) -> float:
        metres = (speed_kmh / 3.6) * BasicBot.BRAKE_LOOKAHEAD_REACTION_S
        return max(BasicBot.BRAKE_LOOKAHEAD_MIN_M, min(BasicBot.BRAKE_LOOKAHEAD_MAX_M, metres))

    def __init__(self) -> None:
        self._tick = 0
        self._csv_writer, self._csv_file = init_debug_csv()
        self.prevIndex = 0
        self._throttle_ctrl = ThrottleController()
        self._gear_ctrl = GearController()
        self._track_guard = TrackGuard()
        self._detrack = DetrackRecovery()

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

        is_turn, _, severity = classify_turn(
            centerline, current_idx, self.brake_lookahead_m(car.speed_kmh), self._tick
        )
        throttle, brake = self._throttle_ctrl.compute(car.speed_kmh, is_turn, severity)

        max_slip = max(
            car.tire_slip.front_left,
            car.tire_slip.front_right,
            car.tire_slip.rear_left,
            car.tire_slip.rear_right,
        )
        if max_slip > self.SLIP_THRESHOLD:
            throttle = 0.0
            brake = max(brake, 0.1)

        track_status = self._track_guard.check(car.position, car.orientation, centerline, current_idx)
        throttle, brake, steering = self._track_guard.apply_recovery(
            track_status, throttle, brake, steering, centerline, current_idx, car.orientation
        )
        throttle, brake, steering, detrack_gs = self._detrack.apply(
            track_status, ctx, throttle, brake, steering, self._tick, snapshot.server_time_ms
        )

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

        gear_shift = self._gear_ctrl.compute(int(car.gear), car.engine_rpm, car.speed_kmh)
        if detrack_gs is not None:
            gear_shift = detrack_gs

        ctx.set_controls(
            throttle=throttle,
            brake=brake,
            steer=steering,
            gear_shift=gear_shift,
        )


if __name__ == "__main__":
    raise SystemExit(run_bot(BasicBot()))