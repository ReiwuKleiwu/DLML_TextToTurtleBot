"""Control loop executing behaviour ticks at a fixed rate."""
from __future__ import annotations

import threading
from typing import Callable


class ControlLoop(threading.Thread):
    """Simple timed thread that repeatedly invokes a callback."""

    def __init__(self, rate_hz: float, tick_callback: Callable[[], None]) -> None:
        super().__init__(name="robot-control-loop", daemon=True)
        self._tick_callback = tick_callback
        self._timeout = 1.0 / rate_hz if rate_hz > 0 else None
        self._done = threading.Event()

    def run(self) -> None:
        while not self._done.is_set():
            self._tick_callback()
            if self._timeout is None:
                self._done.wait()
            else:
                self._done.wait(self._timeout)

    def stop(self) -> None:
        self._done.set()
