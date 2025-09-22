"""Service that reads TF data and publishes robot transform updates."""
from __future__ import annotations

import time
from typing import Optional, Tuple

from code.core.events import EventType
from code.core.interfaces.event_bus import EventBus


class TransformMonitor:
    """Polling helper that routes TF data onto the event bus."""

    def __init__(self, event_bus: EventBus, tf_subscriber) -> None:
        self._bus = event_bus
        self._tf_subscriber = tf_subscriber
        self._robot_position = None
        self._robot_orientation = None
        self._last_update_time = 0.0

    def update_robot_transforms(self) -> None:
        position = self._tf_subscriber.get_position()
        orientation = self._tf_subscriber.get_orientation()
        current_time = time.time()

        self._robot_position = position
        self._robot_orientation = orientation
        self._last_update_time = current_time

        if position is not None and orientation is not None:
            self._bus.publish_event(
                EventType.ROBOT_TRANSFORM_UPDATED,
                source="TFService",
                data={
                    'position': position,
                    'orientation': orientation,
                    'timestamp': current_time,
                }
            )

    def get_robot_pose(self) -> Tuple[Optional[tuple], Optional[tuple]]:
        return self._robot_position, self._robot_orientation

    def is_transform_available(self) -> bool:
        return self._robot_position is not None and self._robot_orientation is not None

    @property
    def last_update_time(self) -> float:
        return self._last_update_time
