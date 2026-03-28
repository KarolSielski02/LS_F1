from __future__ import annotations

from dataclasses import dataclass, field

from braking import BrakeController


@dataclass
class BrakeBenchmark:
    """After `delay_ms` of free running, apply `brake_level` for `duration_ms`."""

    delay_ms: int = 10_000
    brake_level: float = 0.8
    duration_ms: int = 2_000

    _start_ms: int = field(default=-1, init=False)
    _done: bool = field(default=False, init=False)

    def update(
        self,
        controller: BrakeController,
        now_ms: int,
    ) -> None:
        """Call every tick. Drives the controller automatically."""
        if self._done:
            return

        if self._start_ms < 0:
            self._start_ms = now_ms
            return

        elapsed = now_ms - self._start_ms

        if elapsed < self.delay_ms:
            controller.release()
        elif elapsed < self.delay_ms + self.duration_ms:
            controller.request(self.brake_level)
            print(
                f"[benchmark] braking at {self.brake_level:.0%} "
                f"({elapsed - self.delay_ms} / {self.duration_ms} ms)"
            )
        else:
            controller.release()
            self._done = True
            print("[benchmark] brake test complete")
