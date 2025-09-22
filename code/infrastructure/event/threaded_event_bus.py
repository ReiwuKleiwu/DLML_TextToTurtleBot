"""Threaded event bus implementation used by the application."""
from __future__ import annotations

import queue
import threading
from typing import Dict, List

from code.core.events import DomainEvent, EventType
from code.core.interfaces.event_bus import EventBus, EventCallback


class ThreadedEventBus(EventBus):
    """Event bus backed by a worker thread and queue."""

    def __init__(self) -> None:
        self._queue: queue.Queue[DomainEvent] = queue.Queue()
        self._subscribers: Dict[EventType, List[EventCallback]] = {}
        self._lock = threading.Lock()
        self._running = False
        self._worker_thread: threading.Thread | None = None

    def start(self) -> None:
        with self._lock:
            if self._running:
                return
            self._running = True
            self._worker_thread = threading.Thread(
                target=self._process_events,
                name="event-bus-worker",
                daemon=True,
            )
            self._worker_thread.start()

    def stop(self) -> None:
        with self._lock:
            self._running = False
        if self._worker_thread:
            self._worker_thread.join(timeout=1.0)
            self._worker_thread = None

    def subscribe(self, event_type: EventType, callback: EventCallback) -> None:
        with self._lock:
            self._subscribers.setdefault(event_type, []).append(callback)

    def unsubscribe(self, event_type: EventType, callback: EventCallback) -> None:
        with self._lock:
            callbacks = self._subscribers.get(event_type)
            if not callbacks:
                return
            try:
                callbacks.remove(callback)
            except ValueError:
                return

    def publish(self, event: DomainEvent) -> None:
        if not self._running:
            return
        self._queue.put(event)

    def publish_event(self, event_type: EventType, source: str, data: dict) -> None:
        self.publish(DomainEvent(event_type=event_type, source=source, data=data))

    def get_queue_size(self) -> int:
        return self._queue.qsize()

    def _process_events(self) -> None:
        while self._running:
            try:
                event = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                self._dispatch(event)
            finally:
                self._queue.task_done()

    def _dispatch(self, event: DomainEvent) -> None:
        with self._lock:
            callbacks = list(self._subscribers.get(event.event_type, []))
        for callback in callbacks:
            try:
                callback(event)
            except Exception as exc:  # noqa: BLE001 - surface callback failures
                print(f"[EventBus] Error in callback {callback}: {exc}")
