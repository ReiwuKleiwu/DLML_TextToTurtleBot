"""Interface describing how motion commands are emitted."""
from __future__ import annotations

from typing import Protocol


class MotionCommandPublisher(Protocol):
    """Protocol for publishing velocity commands."""

    def publish(self, twist_msg) -> None: ...
