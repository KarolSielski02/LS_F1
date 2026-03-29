from __future__ import annotations

from hackarena3 import BotContext, GearShift

from track_guard import TrackStatus

# Poślizg / poza tor / zła strona — czekaj tyle zanim wywołasz back_to_track (czas na naturalny powrót)
_DELAY_BEFORE_TELEPORT_MS = 5000

# Minimalny odstęp między kolejnymi próbami teleportu, gdy nadal jest źle (ms)
_REPEAT_TELEPORT_MS = 6000


def _call_back_to_track(ctx: BotContext) -> None:
    """Wywołanie serwerowej funkcji ustawiającej auto na torze (nazwa może się różnić w wersji wrappera)."""
    fn = getattr(ctx, "request_back_to_track", None)
    if callable(fn):
        fn()
        return
    fn = getattr(ctx, "back_to_track", None)
    if callable(fn):
        fn()


class DetrackRecovery:
    """Ten sam czas oczekiwania przed teleportem przy poza torem i przy złej stronie w poślizgu."""

    def __init__(self) -> None:
        self._recovery_since_ms: int | None = None
        self._last_teleport_ms: int | None = None

    def apply(
        self,
        status: TrackStatus,
        ctx: BotContext,
        throttle: float,
        brake: float,
        steer: float,
        tick: int,
        server_time_ms: int,
    ) -> tuple[float, float, float, GearShift | None]:
        """Zwraca (gaz, hamulec, skręt, zmiana biegu). ``None`` = użyj ``GearController``."""
        need_help = status.off_track or status.wrong_way
        if not need_help:
            self._recovery_since_ms = None
            self._last_teleport_ms = None
            return throttle, brake, steer, None

        # Pierwszy tick z problemem: start licznika karencji (bez natychmiastowego teleportu)
        if self._recovery_since_ms is None:
            self._recovery_since_ms = server_time_ms

        elapsed = server_time_ms - self._recovery_since_ms
        if elapsed < _DELAY_BEFORE_TELEPORT_MS:
            return throttle, brake, steer, None

        if self._last_teleport_ms is None or server_time_ms - self._last_teleport_ms >= _REPEAT_TELEPORT_MS:
            _call_back_to_track(ctx)
            self._last_teleport_ms = server_time_ms
            if tick % 45 == 0:
                print(f"[Detrack] back_to_track  off={status.off_track} wrong_way={status.wrong_way}")

        return 0.0, 0.0, 0.0, GearShift.NONE
