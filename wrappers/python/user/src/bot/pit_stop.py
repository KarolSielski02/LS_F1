from __future__ import annotations

import math

from hackarena3 import CenterlinePoint, PitstopLayout, Vec3


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
