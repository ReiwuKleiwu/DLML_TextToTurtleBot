"""Application-level coordination reacting to vision events."""
from __future__ import annotations

from typing import Optional, TYPE_CHECKING

from code.core.events import DomainEvent, EventType
from code.core.interfaces.event_bus import EventBus
from code.core.state import RobotState, RobotStateSource
from code.core.state_machine import RobotStateMachine

if TYPE_CHECKING:
    from code.application.services.map_tracker import MapTrackerService


class VisionController:
    """Translates detection updates into state machine transitions."""

    def __init__(
        self,
        event_bus: EventBus,
        state_machine: RobotStateMachine,
        map_tracker: 'MapTrackerService | None' = None,
        target_area_threshold: float = 0.30,
        required_frames: int = 5,
    ) -> None:
        self._bus = event_bus
        self._state_machine = state_machine
        self._target_area_threshold = target_area_threshold
        self._required_frames = required_frames
        self._target_close_counter = 0
        self._map_tracker = map_tracker

        self._bus.subscribe(EventType.OBJECT_DETECTED, self._on_object_detected)
        self._bus.subscribe(EventType.TARGET_REACHED, self._on_target_reached)

    def _on_object_detected(self, event: DomainEvent) -> None:
        current_top = self._state_machine.current_state
        if current_top and current_top.value == RobotState.NAVIGATE:
            return
        requested_target = self._get_requested_target()
        selected_target = event.data.get('selected_target_info')
        camera_width = event.data.get('camera_width')
        camera_height = event.data.get('camera_height')

        if requested_target is None:
            self._ensure_object_found_popped()
            return

        if not selected_target:
            self._target_close_counter = 0
            if self._map_tracker:
                self._map_tracker.mark_selected_target(None, None)
            self._ensure_object_found_popped()
            return

        if camera_width is None or camera_height is None:
            return

        target_data = {
            "object_found_data": {
                "camera": {
                    "width": camera_width,
                    "height": camera_height,
                },
                "class": requested_target,
                "bounding_box_coordinates": {
                    "x1": selected_target['x1'],
                    "y1": selected_target['y1'],
                    "x2": selected_target['x2'],
                    "y2": selected_target['y2'],
                },
            }
        }

        if self._is_target_close(selected_target, camera_width, camera_height):
            self._target_close_counter += 1
        else:
            self._target_close_counter = 0

        if self._map_tracker:
            tracking_id = event.data.get('selected_target_tracking_id')
            self._map_tracker.mark_selected_target(requested_target, tracking_id)

        current = self._state_machine.current_state
        if current and current.value == RobotState.OBJECT_FOUND:
            current.set_data(target_data)
        else:
            self._state_machine.push_state(
                RobotState.OBJECT_FOUND,
                RobotStateSource.CAMERA,
                data=target_data,
            )

    def _on_target_reached(self, event: DomainEvent) -> None:
        self._target_close_counter = 0

    def _ensure_object_found_popped(self) -> None:
        current = self._state_machine.current_state
        if current and current.value == RobotState.OBJECT_FOUND:
            self._state_machine.pop_state(RobotState.OBJECT_FOUND, RobotStateSource.CAMERA)

    def _get_requested_target(self) -> Optional[str]:
        state = self._state_machine.find_state(RobotState.EXPLORE)
        if not state:
            return None
        return state.data.get("target_object")

    def _is_target_close(self, target_info: dict, width: int, height: int) -> bool:
        box_width = target_info['x2'] - target_info['x1']
        box_height = target_info['y2'] - target_info['y1']
        box_area = box_width * box_height
        image_area = width * height
        return (box_area / image_area) > self._target_area_threshold
