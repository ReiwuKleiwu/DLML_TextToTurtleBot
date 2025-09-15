import threading
import queue
import time
from typing import Any, Dict, List, Callable, Optional
from dataclasses import dataclass
from enum import Enum


class EventType(Enum):
    # Camera and vision events
    OBJECT_DETECTED = "object_detected"
    DEPTH_DATA_UPDATED = "depth_data_updated"
    WORLD_COORDINATES_CALCULATED = "world_coordinates_calculated"
    TARGET_SELECTED = "target_selected"
    CAMERA_FRAME_PROCESSED = "camera_frame_processed"

    # LiDAR events
    LIDAR_SCAN_PROCESSED = "lidar_scan_processed"
    OBSTACLE_DETECTED = "obstacle_detected"
    OBSTACLE_CLEARED = "obstacle_cleared"

    # IR sensor events
    IR_OBSTACLE_DETECTED = "ir_obstacle_detected"
    IR_OBSTACLE_CLEARED = "ir_obstacle_cleared"

    # State machine events
    STATE_CHANGED = "state_changed"
    STATE_PUSHED = "state_pushed"
    STATE_POPPED = "state_popped"

    # Navigation events
    TARGET_REACHED = "target_reached"
    NAVIGATION_STARTED = "navigation_started"
    NAVIGATION_STOPPED = "navigation_stopped"

    # System events
    SENSOR_DATA_UPDATED = "sensor_data_updated"
    VISUALIZATION_UPDATE = "visualization_update"


@dataclass
class Event:
    event_type: EventType
    source: str
    data: Dict[str, Any]
    timestamp: float = None

    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = time.time()


class EventQueue:
    _instance = None
    _lock = threading.Lock()

    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super(EventQueue, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        if hasattr(self, '_initialized'):
            return
        self._initialized = True

        self._event_queue = queue.Queue()
        self._subscribers: Dict[EventType, List[Callable]] = {}
        self._running = False
        self._worker_thread = None
        self._lock = threading.Lock()

    def start(self):
        """Start the event processing worker thread"""
        with self._lock:
            if not self._running:
                self._running = True
                self._worker_thread = threading.Thread(target=self._process_events, daemon=True)
                self._worker_thread.start()

    def stop(self):
        """Stop the event processing worker thread"""
        with self._lock:
            self._running = False
            if self._worker_thread:
                self._worker_thread.join(timeout=1.0)

    def subscribe(self, event_type: EventType, callback: Callable[[Event], None]):
        """Subscribe to events of a specific type"""
        with self._lock:
            if event_type not in self._subscribers:
                self._subscribers[event_type] = []
            self._subscribers[event_type].append(callback)

    def unsubscribe(self, event_type: EventType, callback: Callable[[Event], None]):
        """Unsubscribe from events of a specific type"""
        with self._lock:
            if event_type in self._subscribers:
                try:
                    self._subscribers[event_type].remove(callback)
                except ValueError:
                    pass

    def publish(self, event: Event):
        """Publish an event to the queue"""
        if self._running:
            self._event_queue.put(event)

    def publish_event(self, event_type: EventType, source: str, data: Dict[str, Any]):
        """Convenience method to publish an event"""
        event = Event(event_type=event_type, source=source, data=data)
        self.publish(event)

    def _process_events(self):
        """Worker thread method to process events"""
        while self._running:
            try:
                event = self._event_queue.get(timeout=0.1)
                self._dispatch_event(event)
                self._event_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[EventQueue] Error processing event: {e}")

    def _dispatch_event(self, event: Event):
        """Dispatch an event to all subscribers"""
        with self._lock:
            subscribers = self._subscribers.get(event.event_type, [])

        for callback in subscribers:
            try:
                callback(event)
            except Exception as e:
                print(f"[EventQueue] Error in event callback: {e}")

    def get_queue_size(self) -> int:
        """Get the current size of the event queue"""
        return self._event_queue.qsize()

    def clear_subscribers(self, event_type: Optional[EventType] = None):
        """Clear all subscribers for a specific event type or all event types"""
        with self._lock:
            if event_type:
                self._subscribers[event_type] = []
            else:
                self._subscribers.clear()