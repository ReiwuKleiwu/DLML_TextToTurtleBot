"""Contract for the application wide event bus."""
from __future__ import annotations

from typing import Callable, Protocol

from code.core.events import DomainEvent, EventType


EventCallback = Callable[[DomainEvent], None]


class EventBus(Protocol):
    """Protocol describing our event bus abstraction."""

    def start(self) -> None: ...

    def stop(self) -> None: ...

    def subscribe(self, event_type: EventType, callback: EventCallback) -> None: ...

    def unsubscribe(self, event_type: EventType, callback: EventCallback) -> None: ...

    def publish(self, event: DomainEvent) -> None: ...

    def publish_event(self, event_type: EventType, source: str, data: dict) -> None: ...

    def get_queue_size(self) -> int: ...
