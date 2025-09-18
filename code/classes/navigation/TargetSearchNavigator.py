import threading
from typing import Any, Dict, List, Optional, Tuple

from rclpy.node import Node

from classes.controllers.StateMachine import (
    StateMachine,
    TurtleBotState,
    TurtleBotStateSource,
)
from classes.events import Event, EventQueue, EventType
from classes.navigation.SLAMNavigationService import SLAMNavigationService


class TargetSearchNavigator:
    """Coordinate Nav2 goals for targets located via the persistent map."""

    def __init__(
        self,
        node: Node,
        state_machine: StateMachine,
        navigation_service: SLAMNavigationService,
    ) -> None:
        self.node = node
        self.state_machine = state_machine
        self.navigation_service = navigation_service

        self.event_queue = EventQueue()

        self._lock = threading.Lock()
        self._current_target_class: Optional[str] = None
        self._navigation_goal_sent = False
        self._active_goal_coords: Optional[Tuple[float, float, float]] = None
        self._latest_map: Dict[str, List[Dict[str, Any]]] = {}
        self._session_counter = 0
        self._active_session_id: Optional[int] = None

        # Subscribe to relevant events
        self.event_queue.subscribe(EventType.STATE_PUSHED, self._on_state_pushed)
        self.event_queue.subscribe(EventType.STATE_POPPED, self._on_state_popped)
        self.event_queue.subscribe(EventType.SENSOR_DATA_UPDATED, self._on_map_updated)
        self.event_queue.subscribe(EventType.TARGET_REACHED, self._on_target_reached)

    def _on_state_pushed(self, event: Event) -> None:
        if event.data.get('new_state') != 'EXPLORE':
            return

        target_class = self._extract_target_from_event(event)
        if not target_class:
            return

        self._start_new_target_session(target_class)

    def _on_state_popped(self, event: Event) -> None:
        if event.data.get('popped_state') != 'EXPLORE':
            return

        if event.data.get('popped_state_source') and event.data.get('popped_state_source') != 'USER':
            return

        self._cancel_active_navigation('exploration state popped')
        self._activate_current_explore_target()

    def _on_map_updated(self, event: Event) -> None:
        map_data = event.data.get('persistent_objects_map', {}) or {}

        with self._lock:
            self._latest_map = map_data
            should_attempt = (
                self._current_target_class is not None
                and not self._navigation_goal_sent
                and self._active_session_id is not None
                and self._is_target_state_active_locked()
            )

        if should_attempt:
            self._attempt_navigation_goal(map_data)

    def _on_target_reached(self, event: Event) -> None:
        if event.source != 'TargetReachedService':
            return

        self._cancel_active_navigation('target reached')

    def _start_new_target_session(self, target_class: str) -> None:
        cancel_required = False

        with self._lock:
            if target_class == self._current_target_class and self._navigation_goal_sent:
                return

            if self._navigation_goal_sent:
                cancel_required = True

            self._session_counter += 1
            self._active_session_id = self._session_counter
            self._current_target_class = target_class
            self._navigation_goal_sent = False
            self._active_goal_coords = None
            map_snapshot = self._latest_map
            is_active = self._is_target_state_active_locked()

        if cancel_required:
            self.node.get_logger().info(
                'Canceling active Nav2 goal to pursue new target request.'
            )
            self.navigation_service.cancel_navigation()

        if not is_active:
            self.node.get_logger().debug(
                'Skipping stored-target lookup because EXPLORE state is not active on top of the stack.'
            )
            return

        self.node.get_logger().info(
            f'Checking stored map entries for target "{target_class}".'
        )
        self._attempt_navigation_goal(map_snapshot)

    def _cancel_active_navigation(self, reason: str) -> None:
        should_cancel = False

        with self._lock:
            if self._navigation_goal_sent:
                should_cancel = True
            self._reset_tracking_locked()

        if should_cancel:
            self.node.get_logger().info(
                f'Canceling Nav2 goal because {reason}.'
            )
            self.navigation_service.cancel_navigation()

    def _attempt_navigation_goal(
        self,
        map_snapshot: Optional[Dict[str, List[Dict[str, Any]]]],
    ) -> None:
        if not map_snapshot:
            return

        with self._lock:
            if self._navigation_goal_sent or not self._current_target_class:
                return

            target_class = self._current_target_class
            session_id = self._active_session_id

            if not self._is_target_state_active_locked():
                return

            target_objects = map_snapshot.get(target_class, [])
            target_obj = self._select_target_object(target_objects)

            if not target_obj:
                return

            coords = target_obj.get('world_coords')
            if not self._coords_valid(coords):
                return

            x = float(coords[0])
            y = float(coords[1])
            z = float(coords[2]) if len(coords) > 2 and coords[2] is not None else 0.0

            self._navigation_goal_sent = True
            self._active_goal_coords = (x, y, z)

        self.node.get_logger().info(
            f'Publishing Nav2 goal for stored target "{target_class}" at ({x:.2f}, {y:.2f}).'
        )

        success = self.navigation_service.navigate_to_pose(x, y, 0.0)
        if not success:
            with self._lock:
                if self._active_session_id == session_id:
                    self._navigation_goal_sent = False
                    self._active_goal_coords = None
            self.node.get_logger().warn(
                'Unable to send Nav2 goal: NavigateToPose action server not available.'
            )
            return

        with self._lock:
            if self._active_session_id != session_id:
                self._navigation_goal_sent = False
                self._active_goal_coords = None
                cancel_required = True
            else:
                cancel_required = False

        if cancel_required:
            self.navigation_service.cancel_navigation()

    def _select_target_object(
        self, objects: List[Dict[str, Any]]
    ) -> Optional[Dict[str, Any]]:
        prioritized = [
            obj for obj in objects
            if obj.get('is_selected_target') and self._coords_valid(obj.get('world_coords'))
        ]
        if prioritized:
            return prioritized[0]

        valid_objects = [
            obj for obj in objects
            if self._coords_valid(obj.get('world_coords'))
        ]
        if not valid_objects:
            return None

        return max(valid_objects, key=lambda obj: obj.get('last_seen', 0.0))

    def _coords_valid(self, coords: Any) -> bool:
        if not isinstance(coords, (list, tuple)):
            return False
        if len(coords) < 2:
            return False
        if coords[0] is None or coords[1] is None:
            return False
        return True

    def _reset_tracking_locked(self) -> None:
        self._current_target_class = None
        self._navigation_goal_sent = False
        self._active_goal_coords = None
        self._active_session_id = None

    def _activate_current_explore_target(self) -> None:
        current_state = self.state_machine.get_current_state()
        if not current_state:
            return

        if (
            current_state.value != TurtleBotState.EXPLORE
            or current_state.source != TurtleBotStateSource.USER
        ):
            return

        target_class = None
        if isinstance(current_state.data, dict):
            target_class = current_state.data.get('target_object')

        if not target_class:
            return

        self._start_new_target_session(target_class)

    def _is_target_state_active_locked(self) -> bool:
        current_state = self.state_machine.get_current_state()
        if not current_state:
            return False

        if (
            current_state.value != TurtleBotState.EXPLORE
            or current_state.source != TurtleBotStateSource.USER
        ):
            return False

        if not isinstance(current_state.data, dict):
            return False

        return current_state.data.get('target_object') == self._current_target_class

    def _extract_target_from_event(self, event: Event) -> Optional[str]:
        if event.data.get('new_state_source') and event.data.get('new_state_source') != 'USER':
            return None

        state_data = event.data.get('state_data')
        if isinstance(state_data, dict):
            target = state_data.get('target_object')
            if target:
                return target

        current_state = self.state_machine.get_current_state()
        if self._is_target_state_active_locked():
            data = current_state.data or {}
            return data.get('target_object')

        return None
