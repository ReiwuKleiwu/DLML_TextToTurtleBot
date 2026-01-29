"""Synchronous event bus implementation used by the application."""
from __future__ import annotations

from collections import deque
from typing import Callable, Deque, Dict, List

from shared.utils.singleton_meta import SingletonMeta
from shared.events.interfaces.events import DomainEvent, EventType

EventCallback = Callable[[DomainEvent], None]


class EventBus(metaclass=SingletonMeta):
    """Singleton Event bus delivering events synchronously without worker threads."""

    def __init__(self) -> None:
        self._subscribers: Dict[EventType, List[EventCallback]] = {}
        self._queue: Deque[DomainEvent] = deque()

    def start(self) -> None:
        """Kept for API compatibility; no threaded work to start."""

    def stop(self) -> None:
        """Kept for API compatibility; no threaded work to stop."""

    def subscribe(self, event_type: EventType, callback: EventCallback) -> None:
        self._subscribers.setdefault(event_type, []).append(callback)

    def unsubscribe(self, event_type: EventType, callback: EventCallback) -> None:
        callbacks = self._subscribers.get(event_type)
        if not callbacks:
            return
        try:
            callbacks.remove(callback)
        except ValueError:
            return

    def publish(self, event: DomainEvent) -> None:
        self._queue.append(event)
        self._process_pending()

    def _process_pending(self) -> None:
        """Dispatch all queued events synchronously."""
        while self._queue:
            event = self._queue.popleft()
            self._dispatch(event)

    def get_queue_size(self) -> int:
        return len(self._queue)

    def _dispatch(self, event: DomainEvent) -> None:
        for callback in list(self._subscribers.get(event.event_type, [])):
            try:
                callback(event)
            except Exception as exc:  # noqa: BLE001 - surface callback failures
                print(f"[EventBus] Error in callback {callback}: {exc}")
