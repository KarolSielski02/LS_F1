from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from hackarena3 import BotContext, RaceSnapshot, CenterlinePoint

from hackarena3 import CenterlinePoint

DEBUG_CSV_PATH = Path("bot_debug.csv")

DEBUG_FIELDNAMES = [
    "tick", "server_time_ms", "s_m", "speed_mps", "speed_kmh",
    "gear", "engine_rpm", "pos_x", "pos_y", "pos_z",
    "curvature_1pm", "grade_rad", "bank_rad",
    "lookahead_m", "lookahead_s_m", "lookahead_curvature_1pm",
    "brake_lookahead_m", "brake_lookahead_s_m", "brake_lookahead_curvature_1pm",
    "steering", "throttle", "brake",
    "slip_front_left", "slip_front_right", "slip_rear_left", "slip_rear_right", "slip_max",
    "wear_front_left", "wear_front_right", "wear_rear_left", "wear_rear_right", "wear_min",
    "temp_front_left_celsius", "temp_front_right_celsius",
    "temp_rear_left_celsius", "temp_rear_right_celsius",
    "ghost_phase", "ghost_can_collide", "pit_request_active", "pit_zone_flags",
]


def init_debug_csv() -> tuple[csv.DictWriter, object]:
    """Tworzy (lub nadpisuje) plik CSV i zwraca (writer, file handle)."""
    f = DEBUG_CSV_PATH.open("w", newline="", encoding="utf-8")
    writer = csv.DictWriter(f, fieldnames=DEBUG_FIELDNAMES)
    writer.writeheader()
    return writer, f


def _get_lookahead_point(
    centerline: tuple[CenterlinePoint, ...],
    current_idx: int,
    lookahead_m: float,
) -> CenterlinePoint:
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


def write_debug_row(
    writer: csv.DictWriter,
    snapshot: RaceSnapshot,
    ctx: BotContext,
    current_idx: int,
    current_point: CenterlinePoint,
    lookahead_m: float,
    brake_lookahead_m: float,
    steering: float,
    throttle: float,
    brake: float,
    max_slip: float,
    min_wear: float,
) -> None:
    car = snapshot.car
    centerline = ctx.track.centerline

    lookahead_point = _get_lookahead_point(centerline, current_idx, lookahead_m)
    brake_point = _get_lookahead_point(centerline, current_idx, brake_lookahead_m)

    writer.writerow({
        "tick":                          snapshot.tick,
        "server_time_ms":                snapshot.server_time_ms,
        "s_m":                           round(current_point.s_m, 3),
        "speed_mps":                     round(car.speed_mps, 4),
        "speed_kmh":                     round(car.speed_mps * 3.6, 4),
        "gear":                          car.gear.name,
        "engine_rpm":                    round(car.engine_rpm, 1),
        "pos_x":                         round(car.position.x, 4),
        "pos_y":                         round(car.position.y, 4),
        "pos_z":                         round(car.position.z, 4),
        "curvature_1pm":                 round(current_point.curvature_1pm, 6),
        "grade_rad":                     round(current_point.grade_rad, 6),
        "bank_rad":                      round(current_point.bank_rad, 6),
        "lookahead_m":                   round(lookahead_m, 3),
        "lookahead_s_m":                 round(lookahead_point.s_m, 3),
        "lookahead_curvature_1pm":       round(lookahead_point.curvature_1pm, 6),
        "brake_lookahead_m":             round(brake_lookahead_m, 3),
        "brake_lookahead_s_m":           round(brake_point.s_m, 3),
        "brake_lookahead_curvature_1pm": round(brake_point.curvature_1pm, 6),
        "steering":                      round(steering, 4),
        "throttle":                      round(throttle, 4),
        "brake":                         round(brake, 4),
        "slip_front_left":               round(car.tire_slip.front_left, 4),
        "slip_front_right":              round(car.tire_slip.front_right, 4),
        "slip_rear_left":                round(car.tire_slip.rear_left, 4),
        "slip_rear_right":               round(car.tire_slip.rear_right, 4),
        "slip_max":                      round(max_slip, 4),
        "wear_front_left":               round(car.tire_wear.front_left, 4),
        "wear_front_right":              round(car.tire_wear.front_right, 4),
        "wear_rear_left":                round(car.tire_wear.rear_left, 4),
        "wear_rear_right":               round(car.tire_wear.rear_right, 4),
        "wear_min":                      round(min_wear, 4),
        "temp_front_left_celsius":       round(car.tire_temperature_celsius.front_left_celsius, 2),
        "temp_front_right_celsius":      round(car.tire_temperature_celsius.front_right_celsius, 2),
        "temp_rear_left_celsius":        round(car.tire_temperature_celsius.rear_left_celsius, 2),
        "temp_rear_right_celsius":       round(car.tire_temperature_celsius.rear_right_celsius, 2),
        "ghost_phase":                   car.ghost_mode.phase.name,
        "ghost_can_collide":             car.ghost_mode.can_collide_now,
        "pit_request_active":            car.pit_request_active,
        "pit_zone_flags":                str(car.pitstop_zone_flags),
    })