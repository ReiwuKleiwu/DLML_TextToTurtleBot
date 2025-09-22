"""IR sensor processing that emits obstacle events."""
from __future__ import annotations

from code.core.events import EventType
from code.core.interfaces.event_bus import EventBus


class IRProcessor:
    """Publish IR obstacle events to the event bus."""

    def __init__(self, event_bus: EventBus, min_distance: int = 175) -> None:
        self._bus = event_bus
        self._min_distance = min_distance
        self._obstacle_detected = False

    def handle(self, ir_intensity_vector) -> None:
        obstacle_detected = any(reading.value > self._min_distance for reading in ir_intensity_vector.readings)

        if obstacle_detected and not self._obstacle_detected:
            max_intensity = max(reading.value for reading in ir_intensity_vector.readings)
            self._bus.publish_event(
                EventType.IR_OBSTACLE_DETECTED,
                source="IRProcessor",
                data={
                    'max_intensity': max_intensity,
                    'min_distance': self._min_distance,
                    'sensor_type': 'ir',
                    'readings': [reading.value for reading in ir_intensity_vector.readings],
                },
            )
            self._obstacle_detected = True
        elif not obstacle_detected and self._obstacle_detected:
            self._bus.publish_event(
                EventType.IR_OBSTACLE_CLEARED,
                source="IRProcessor",
                data={'sensor_type': 'ir'},
            )
            self._obstacle_detected = False
