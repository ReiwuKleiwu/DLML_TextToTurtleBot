"""Runtime control facade for LLM or external agents."""
from __future__ import annotations

import threading
import time
from typing import Any, Dict, List, Optional

import math

from rclpy.impl.rcutils_logger import RcutilsLogger

from code.application.controllers.navigation_controller import TargetSearchNavigator
from code.application.runtime.orchestrator import TurtleBotOrchestrator
from code.application.services.map_tracker import MapTrackerService
from code.application.services.tf_monitor import TransformMonitor
from code.infrastructure.visualization.visualization_service import VisualizationService
from code.core.events import EventType
from code.core.interfaces.event_bus import EventBus
from code.core.state import RobotState, RobotStateSource
from code.core.state_machine import RobotStateMachine
from code.infrastructure.perception.camera_processor import CameraProcessor
from code.infrastructure.ros.navigation.slam_navigation_service import SLAMNavigationService


class RobotControlAPI:
    """Expose high-level robot controls and status for LLM-driven workflows."""

    def __init__(
        self,
        *,
        state_machine: RobotStateMachine,
        orchestrator: TurtleBotOrchestrator,
        camera_processor: CameraProcessor,
        navigation_service: SLAMNavigationService,
        target_navigator: TargetSearchNavigator,
        map_service: MapTrackerService,
        visualization_service: VisualizationService,
        transform_monitor: TransformMonitor,
        event_bus: EventBus,
        logger: RcutilsLogger,
    ) -> None:
        self._state_machine = state_machine
        self._orchestrator = orchestrator
        self._camera_processor = camera_processor
        self._navigation_service = navigation_service
        self._target_navigator = target_navigator
        self._map_service = map_service
        self._visualization_service = visualization_service
        self._transform_monitor = transform_monitor
        self._event_bus = event_bus
        self._logger = logger

        self._lock = threading.Lock()
        self._event_bus.subscribe(EventType.NAVIGATION_STOPPED, self._on_navigation_stopped)
        self._active_nav_goal: Optional[Dict[str, float]] = None

    # ------------------------------------------------------------------
    # Command primitives
    # ------------------------------------------------------------------
    def request_target(self, target_class: str) -> None:
        """Queue a new target search request."""
        target = self._normalise_target(target_class)
        self._logger.info(f"LLM control: requesting target '{target}'")

        with self._lock:
            plan = self._get_user_plan()
            plan.append(('explore', target))
            self._set_user_plan(plan)

    def cancel_current_target(self) -> bool:
        """Cancel the current EXPLORE target if one is active."""
        with self._lock:
            plan = self._get_user_plan()
            for index, (kind, _) in enumerate(plan):
                if kind == 'explore':
                    del plan[index]
                    self._set_user_plan(plan)
                    return True
        return False

    def clear_all_targets(self) -> None:
        """Remove all pending user targets and stop navigation."""
        self._logger.info("LLM control: clearing all targets")
        with self._lock:
            self._logger.info("🚫 NAV2 ABORT SOURCE: LLM control_api.clear_all_targets() - User/LLM requested clearing all targets")
            self._navigation_service.cancel_navigation()
            self._target_navigator.clear_targets()
            self._camera_processor.reset_targets()
            self._map_service.mark_selected_target(None, None)
            self._set_user_plan([])

    def reset_state_stack(self) -> None:
        """Reset the state machine and related controllers back to IDLE."""
        self._logger.info("LLM control: resetting state stack to IDLE")
        with self._lock:
            self._logger.info("🚫 NAV2 ABORT SOURCE: LLM control_api.reset_state_stack() - User/LLM requested state reset")
            self._navigation_service.cancel_navigation()
            self._target_navigator.clear_targets()
            self._camera_processor.reset_targets()
            self._state_machine.reset()
            self._map_service.mark_selected_target(None, None)
            self._active_nav_goal = None
            self._set_user_plan([])

    def get_visualization_snapshot(self) -> Dict[str, Any]:
        """Return the latest visualization view model."""
        return self._visualization_service.get_view_model()

    def clear_map(self) -> None:
        """Clear persistent map data and reset visualization trajectory."""
        self._map_service.clear_map()
        self._visualization_service.clear_trajectory()
        with self._lock:
            self._set_user_plan(self._get_user_plan())

    def get_camera_frame(self) -> Optional[bytes]:
        """Fetch the most recent RGB camera frame (JPEG bytes)."""
        return self._camera_processor.get_latest_frame()

    # ------------------------------------------------------------------
    # Target plan helpers
    # ------------------------------------------------------------------
    def _get_user_plan(self) -> List[tuple[str, Any]]:
        """Return the user-defined plan in execution order (current task first)."""
        plan: List[tuple[str, Any]] = []
        for snapshot in reversed(self._state_machine.get_state_stack()):
            if snapshot.source != RobotStateSource.USER:
                continue
            if snapshot.value == RobotState.EXPLORE:
                target_object = snapshot.data.get('target_object') if snapshot.data else None
                if target_object:
                    plan.append(('explore', self._normalise_target(target_object)))
            elif snapshot.value == RobotState.NAVIGATE:
                goal = snapshot.data.get('goal') if snapshot.data else None
                if goal:
                    plan.append(('navigate', goal))
        return plan

    def _set_user_plan(self, plan: List[tuple[str, Any]]) -> None:
        """Apply the provided plan to the state machine."""

        def plan_key(task_type: str, payload: Any) -> tuple:
            if task_type == 'explore':
                return ('explore', payload)
            if task_type == 'navigate':
                goal = payload or {}
                return (
                    'navigate',
                    float(goal.get('x', 0.0)),
                    float(goal.get('y', 0.0)),
                    float(goal.get('yaw', 0.0)),
                )
            return (task_type, payload)

        existing_snapshots: Dict[tuple, Any] = {}
        for snapshot in reversed(self._state_machine.get_state_stack()):
            if snapshot.source != RobotStateSource.USER:
                continue
            if snapshot.value not in (RobotState.EXPLORE, RobotState.NAVIGATE):
                continue
            key = None
            if snapshot.value == RobotState.EXPLORE:
                target_object = snapshot.data.get('target_object') if snapshot.data else None
                if target_object:
                    key = plan_key('explore', self._normalise_target(target_object))
            elif snapshot.value == RobotState.NAVIGATE:
                goal = snapshot.data.get('goal') if snapshot.data else None
                if goal:
                    key = plan_key('navigate', goal)
            if key and key not in existing_snapshots:
                existing_snapshots[key] = dict(snapshot.data) if snapshot.data else {}

        while True:
            current = self._state_machine.current_state
            if not current or current.value == RobotState.IDLE:
                break
            if current.value == RobotState.OBJECT_FOUND and current.source == RobotStateSource.CAMERA:
                self._state_machine.pop_state(RobotState.OBJECT_FOUND, RobotStateSource.CAMERA)
                continue
            if current.source == RobotStateSource.USER and current.value in (RobotState.EXPLORE, RobotState.NAVIGATE):
                self._state_machine.pop_state(current.value, RobotStateSource.USER)
                continue
            break

        for task_type, payload in reversed(plan):
            key = plan_key(task_type, payload)
            existing_data = existing_snapshots.get(key)

            if task_type == 'explore':
                data = dict(existing_data) if existing_data is not None else {'target_object': payload}
                data.setdefault('target_object', payload)
                self._state_machine.push_state(
                    RobotState.EXPLORE,
                    RobotStateSource.USER,
                    data,
                )
            elif task_type == 'navigate':
                data = dict(existing_data) if existing_data is not None else {
                    'goal': payload,
                    'status': 'pending',
                    'requested_at': time.time(),
                }
                data.setdefault('goal', payload)
                data.setdefault('status', 'pending')
                data.setdefault('requested_at', time.time())
                self._state_machine.push_state(
                    RobotState.NAVIGATE,
                    RobotStateSource.USER,
                    data,
                )

        self._set_camera_target_from_plan(plan)
        self._unblock_navigation_if_needed()

    def _set_camera_target_from_plan(self, plan: List[tuple[str, Any]]) -> None:
        for task_type, payload in plan:
            if task_type == 'explore':
                self._camera_processor.set_target_object(payload)
                return
        self._camera_processor.set_target_object(None)

    def _unblock_navigation_if_needed(self) -> None:
        stack = self._state_machine.get_state_stack()
        if len(stack) < 2:
            return
        top = stack[-1]
        below = stack[-2]
        if (
            top.value == RobotState.OBJECT_FOUND
            and top.source == RobotStateSource.CAMERA
            and below.value == RobotState.NAVIGATE
            and below.source == RobotStateSource.USER
        ):
            self._state_machine.pop_state(RobotState.OBJECT_FOUND, RobotStateSource.CAMERA)

    def queue_nav_goal(self, x: float, y: float, yaw: Optional[float] = None) -> bool:
        """Send a manual Nav2 goal."""
        x_val = float(x)
        y_val = float(y)
        if yaw is not None:
            goal_yaw = float(yaw)
        else:
            position, _ = self._transform_monitor.get_robot_pose()
            if position and len(position) >= 2:
                try:
                    x_curr = float(position[0])
                    y_curr = float(position[1])
                    goal_yaw = math.atan2(y_val - y_curr, x_val - x_curr)
                except (TypeError, ValueError):
                    goal_yaw = 0.0
            else:
                goal_yaw = 0.0
        goal_payload = {'x': x_val, 'y': y_val, 'yaw': goal_yaw}
        self._logger.info(
            f"LLM control: queueing navigation goal to ({x_val:.2f}, {y_val:.2f}, yaw={goal_yaw:.2f})"
        )

        with self._lock:
            plan = self._get_user_plan()
            plan.append(('navigate', goal_payload))
            self._set_user_plan(plan)
        return True

    def cancel_nav_goal(self) -> None:
        """Cancel any active Nav2 goal."""
        self._logger.info("LLM control: canceling navigation goal")
        self._logger.info("🚫 NAV2 ABORT SOURCE: LLM control_api.cancel_nav_goal() - User/LLM explicitly cancelled navigation")
        self._navigation_service.cancel_navigation()
        self._pop_nav_state()

    def pause_execution(self) -> None:
        """Pause the orchestrator so behaviours stop executing."""
        if not self._orchestrator.is_paused():
            self._logger.info("LLM control: pausing orchestrator")
            self._orchestrator.pause()
        self._logger.info("🚫 NAV2 ABORT SOURCE: LLM control_api.pause_execution() - User/LLM paused execution")
        self._navigation_service.cancel_navigation()

    def resume_execution(self) -> None:
        """Resume the orchestrator after stack edits."""
        if self._orchestrator.is_paused():
            self._logger.info("LLM control: resuming orchestrator")
            self._orchestrator.resume()

    # ------------------------------------------------------------------
    # Snapshot
    # ------------------------------------------------------------------
    def get_snapshot(self) -> Dict[str, Any]:
        """Return a structured view of the robot state for LLM consumption."""
        stack = [
            {
                'state': snapshot.value.name,
                'source': snapshot.source.name,
                'data': snapshot.data,
            }
            for snapshot in self._state_machine.get_state_stack()
        ]

        target_queue = [
            snapshot.data.get('target_object')
            for snapshot in self._state_machine.get_state_stack()
            if snapshot.value == RobotState.EXPLORE and snapshot.source == RobotStateSource.USER
        ]

        nav_goal = self._navigation_service.get_last_goal()
        nav_status = self._navigation_service.get_last_status()
        nav_info = {
            'active': self._navigation_service.is_navigation_active(),
            'current_goal': self._format_goal(nav_goal),
            'status': nav_status.get('status'),
            'status_code': nav_status.get('status_code'),
        }

        robot_position, robot_orientation = self._transform_monitor.get_robot_pose()

        snapshot: Dict[str, Any] = {
            'timestamp': time.time(),
            'state_stack': stack,
            'current_state': stack[-1] if stack else None,
            'target_queue': target_queue,
            'current_target': target_queue[-1] if target_queue else None,
            'navigation': nav_info,
            'paused': self._orchestrator.is_paused(),
            'robot_pose': {
                'position': robot_position,
                'orientation': robot_orientation,
                'last_update_time': self._transform_monitor.last_update_time,
            },
            'persistent_objects': self._map_service.get_persistent_objects_snapshot(),
            'event_queue_depth': self._event_bus.get_queue_size(),
        }
        return snapshot

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _normalise_target(self, target: str) -> str:
        if not isinstance(target, str):
            raise ValueError('Target must be a string')
        cleaned = target.strip()
        if not cleaned:
            raise ValueError('Target must not be empty')
        return cleaned

    def _format_goal(self, goal: Optional[tuple]) -> Optional[Dict[str, float]]:
        if goal is None:
            return None
        x, y, yaw = goal
        return {'x': x, 'y': y, 'yaw': yaw}

    def _pop_nav_state(self) -> None:
        current = self._state_machine.current_state
        if current and current.value == RobotState.NAVIGATE and current.source == RobotStateSource.USER:
            self._state_machine.pop_state(RobotState.NAVIGATE, RobotStateSource.USER)
        self._active_nav_goal = None

    def _on_navigation_stopped(self, event) -> None:
        self._pop_nav_state()
