"""Visualization service that listens to domain events and provides view models."""
from __future__ import annotations

import copy
import threading
from collections import deque
from typing import Any, Deque, Dict, Iterable, Optional, Tuple

from code.core.events import DomainEvent, EventType
from code.core.interfaces.event_bus import EventBus


class VisualizationService:
    """Collects visualization state for consumption by the web UI."""

    TRAJECTORY_LIMIT = 600

    def __init__(self, event_bus: EventBus) -> None:
        self._bus = event_bus

        self.persistent_objects_map: Dict[str, Any] = {}
        self.lidar_scan_data: Dict[str, Any] = {}
        self.robot_position: Optional[Iterable[float]] = None
        self.robot_orientation: Optional[Iterable[float]] = None
        self.current_target_object: Optional[str] = None
        self.robot_state: str = "UNKNOWN"
        self._nav_goal: Optional[Tuple[float, float]] = None
        self._nav_status: Optional[str] = None
        self._trajectory: Deque[Tuple[float, float]] = deque(maxlen=self.TRAJECTORY_LIMIT)

        self._view_lock = threading.Lock()
        self._latest_view: Dict[str, Any] = {}

        self._state_stack: list[Dict[str, Any]] = [
            {
                'state': 'IDLE',
                'source': 'USER',
                'data': {},
            }
        ]

        self._bus.subscribe(EventType.SENSOR_DATA_UPDATED, self._on_map_updated)
        self._bus.subscribe(EventType.LIDAR_SCAN_PROCESSED, self._on_lidar_updated)
        self._bus.subscribe(EventType.STATE_CHANGED, self._on_state_changed)
        self._bus.subscribe(EventType.STATE_PUSHED, self._on_state_pushed)
        self._bus.subscribe(EventType.STATE_POPPED, self._on_state_popped)
        self._bus.subscribe(EventType.ROBOT_TRANSFORM_UPDATED, self._on_robot_transform_updated)
        self._bus.subscribe(EventType.NAVIGATION_STARTED, self._on_navigation_started)
        self._bus.subscribe(EventType.NAVIGATION_STOPPED, self._on_navigation_stopped)

    def _on_map_updated(self, event: DomainEvent) -> None:
        if event.source == "MapService":
            self.persistent_objects_map = event.data.get('persistent_objects_map', {})
            self._update_visualization()

    def _on_lidar_updated(self, event: DomainEvent) -> None:
        self.lidar_scan_data = {
            'scan_points': event.data.get('scan_points', []),
            'obstacle_points': event.data.get('obstacle_points', []),
            'timestamp': event.data.get('timestamp')
        }
        self._update_visualization()

    def _on_state_changed(self, event: DomainEvent) -> None:
        if event.source == "StateMachine":
            self.robot_state = event.data.get('current_state', "UNKNOWN")
            state_data = event.data.get('state_data', {})
            if state_data and 'target_object' in state_data:
                self.current_target_object = state_data['target_object']
            elif self.robot_state == 'IDLE':
                self.current_target_object = None
                if event.data.get('current_state_source') == 'USER':
                    self._state_stack = [
                        {
                            'state': 'IDLE',
                            'source': 'USER',
                            'data': {},
                        }
                    ]
            self._update_visualization()

    def _on_state_pushed(self, event: DomainEvent) -> None:
        if event.source != "StateMachine":
            return
        new_state = event.data.get('new_state')
        new_source = event.data.get('new_state_source')
        state_data = event.data.get('state_data') or {}
        if not new_state or not new_source:
            return
        self._state_stack.append({
            'state': new_state,
            'source': new_source,
            'data': copy.deepcopy(state_data),
        })
        self._update_visualization()

    def _on_state_popped(self, event: DomainEvent) -> None:
        if event.source != "StateMachine":
            return
        popped_state = event.data.get('popped_state')
        popped_source = event.data.get('popped_state_source')
        if not popped_state or not popped_source:
            return
        if len(self._state_stack) > 1:
            current = self._state_stack[-1]
            if current['state'] == popped_state and current['source'] == popped_source:
                self._state_stack.pop()
        else:
            self._state_stack = [
                {
                    'state': 'IDLE',
                    'source': 'USER',
                    'data': {},
                }
            ]
        self._update_visualization()

    def _on_robot_transform_updated(self, event: DomainEvent) -> None:
        self.robot_position = event.data.get('position')
        self.robot_orientation = event.data.get('orientation')
        self._append_trajectory_point(self.robot_position)
        self._update_visualization()

    def _update_visualization(self) -> None:
        world_object_map = {}
        total_objects = 0

        for object_class, objects_list in self.persistent_objects_map.items():
            world_object_map[object_class] = []
            for obj in objects_list:
                coords = obj.get('world_coords')
                if isinstance(coords, (list, tuple)) and len(coords) >= 3:
                    try:
                        coords = [float(coords[0]), float(coords[1]), float(coords[2])]
                    except (TypeError, ValueError):
                        coords = None
                elif isinstance(coords, (list, tuple)) and len(coords) >= 2:
                    try:
                        coords = [float(coords[0]), float(coords[1])]
                    except (TypeError, ValueError):
                        coords = None
                else:
                    coords = None
                world_object_map[object_class].append({
                    'world_coords': coords,
                    'detection_count': obj.get('detection_count', 0),
                    'last_seen': obj.get('last_seen', 0),
                    'first_seen': obj.get('first_seen', 0),
                    'is_selected_target': obj.get('is_selected_target', False)
                })
                total_objects += 1

        object_counts = {cls: len(objs) for cls, objs in world_object_map.items()}

        additional_info = {
            'object_count': object_counts,
            'robot_state': self.robot_state,
            'target_object': self.current_target_object,
            'total_objects': total_objects,
            'lidar_points': len(self.lidar_scan_data.get('scan_points', [])),
            'nav_status': self._nav_status,
        }

        lidar_points = self.lidar_scan_data.get('scan_points', []) or []
        obstacle_points = self.lidar_scan_data.get('obstacle_points', []) or []

        def _convert_points(points):
            converted = []
            for point in points:
                if isinstance(point, (list, tuple)) and len(point) >= 2:
                    try:
                        converted.append([float(point[0]), float(point[1])])
                    except (TypeError, ValueError):
                        continue
            return converted

        def _convert_pose(value):
            if value is None:
                return None
            if isinstance(value, (list, tuple)):
                try:
                    return [float(component) for component in value]
                except (TypeError, ValueError):
                    return [float(component) for component in value[:2]]
            return value

        timestamp = self.lidar_scan_data.get('timestamp')

        def _normalise_timestamp(value):
            if value is None:
                return None
            if isinstance(value, (int, float)):
                return float(value)
            sec = getattr(value, 'sec', None)
            nanosec = getattr(value, 'nanosec', None)
            if sec is not None and nanosec is not None:
                try:
                    return float(sec) + float(nanosec) / 1_000_000_000.0
                except (TypeError, ValueError):
                    return float(sec)
            return str(value)

        lidar_view = {
            'scan_points': _convert_points(lidar_points),
            'obstacle_points': _convert_points(obstacle_points),
            'timestamp': _normalise_timestamp(timestamp),
        }

        with self._view_lock:
            self._latest_view = {
                'robot': {
                    'position': _convert_pose(self.robot_position),
                    'orientation': _convert_pose(self.robot_orientation),
                    'state': self.robot_state,
                    'target_object': self.current_target_object,
                    'trajectory': [list(point) for point in self._trajectory],
                },
                'objects': world_object_map,
                'lidar': lidar_view,
                'navigation': {
                    'goal': list(self._nav_goal) if self._nav_goal else None,
                    'status': self._nav_status,
                },
                'additional_info': additional_info,
                'state_stack': list(self._state_stack),
            }

    def _append_trajectory_point(self, position: Optional[Iterable[float]]) -> None:
        if not position:
            return
        try:
            x, y = float(position[0]), float(position[1])
        except (TypeError, IndexError, ValueError):
            return
        self._trajectory.append((x, y))

    def get_view_model(self) -> Dict[str, Any]:
        with self._view_lock:
            return copy.deepcopy(self._latest_view)

    def clear_trajectory(self) -> None:
        with self._view_lock:
            self._trajectory.clear()
            if 'robot' in self._latest_view:
                self._latest_view['robot']['trajectory'] = []

    # ------------------------------------------------------------------
    # Navigation events
    # ------------------------------------------------------------------
    def _on_navigation_started(self, event: DomainEvent) -> None:
        goal = event.data.get('goal') if isinstance(event.data, dict) else None
        if isinstance(goal, dict):
            x = goal.get('x')
            y = goal.get('y')
            if isinstance(x, (int, float)) and isinstance(y, (int, float)):
                self._nav_goal = (float(x), float(y))
                self._nav_status = 'ACTIVE'
                self._update_visualization()

    def _on_navigation_stopped(self, event: DomainEvent) -> None:
        status = event.data.get('status') if isinstance(event.data, dict) else None
        self._nav_status = status or 'IDLE'
        self._nav_goal = None
        self._update_visualization()
