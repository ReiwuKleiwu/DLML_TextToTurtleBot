"""Navigation controller that coordinates Nav2 goals for targets from the persistent map."""
from __future__ import annotations

import math
import time
from typing import Any, Dict, List, Optional, Tuple

from code.core.events import DomainEvent, EventType
from code.core.interfaces.event_bus import EventBus
from code.core.state import RobotState, RobotStateSource
from code.core.state_machine import RobotStateMachine


class TargetSearchNavigator:
    """Coordinates autonomous navigation to targets stored in the persistent map."""

    def __init__(
        self,
        event_bus: EventBus,
        state_machine: RobotStateMachine,
        navigation_service,
        logger,
        max_retry_attempts: int = 3,
        retry_delay: float = 2.0,
        target_proximity_threshold: float = 1.0
    ) -> None:
        self._bus = event_bus
        self._state_machine = state_machine
        self._navigation_service = navigation_service
        self._logger = logger

        # Configuration
        self._max_retry_attempts = max_retry_attempts
        self._retry_delay = retry_delay
        self._target_proximity_threshold = target_proximity_threshold

        # State tracking
        self._current_target: Optional[str] = None
        self._navigation_active = False
        self._current_goal_coords: Optional[Tuple[float, float]] = None
        self._retry_count = 0
        self._last_retry_time = 0.0
        self._persistent_map: Dict[str, List[Dict[str, Any]]] = {}
        self._robot_position: Optional[Tuple[float, float]] = None

        # Subscribe to events
        self._bus.subscribe(EventType.STATE_PUSHED, self._on_state_pushed)
        self._bus.subscribe(EventType.STATE_POPPED, self._on_state_popped)
        self._bus.subscribe(EventType.SENSOR_DATA_UPDATED, self._on_map_updated)
        self._bus.subscribe(EventType.TARGET_REACHED, self._on_target_reached)
        self._bus.subscribe(EventType.ROBOT_TRANSFORM_UPDATED, self._on_robot_transform_updated)
        self._bus.subscribe(EventType.NAVIGATION_STOPPED, self._on_navigation_stopped)

    def _on_state_pushed(self, event: DomainEvent) -> None:
        """Handle when a new state is pushed to the state machine."""
        if event.data.get('new_state') != RobotState.EXPLORE.name:
            return

        if event.data.get('new_state_source') != RobotStateSource.USER.name:
            return

        state_data = event.data.get('state_data', {})
        target_object = state_data.get('target_object')

        if not target_object:
            self._logger.warning("EXPLORE state pushed without target_object")
            return

        self._start_navigation_to_target(target_object)

    def _on_state_popped(self, event: DomainEvent) -> None:
        """Handle when a state is popped from the state machine."""
        if event.data.get('popped_state') != RobotState.EXPLORE.name:
            return

        self._logger.info("🚫 NAV2 ABORT SOURCE: navigation_controller._on_state_popped() - EXPLORE state was popped")
        self._cancel_navigation("exploration state ended")

    def _on_map_updated(self, event: DomainEvent) -> None:
        """Handle updates to the persistent map."""
        self._persistent_map = event.data.get('persistent_objects_map', {})

        # If we have a target but no active navigation, try to start navigation
        if self._current_target and not self._navigation_active:
            self._attempt_navigation_to_current_target()

    def _on_target_reached(self, event: DomainEvent) -> None:
        """Handle when target is reached by vision system."""
        if event.source != 'TargetReachedService':
            return

        self._logger.info("Target reached by vision system, stopping navigation")
        self._logger.info("🚫 NAV2 ABORT SOURCE: navigation_controller._on_target_reached() - Vision system detected target reached")
        self._cancel_navigation("target reached")
        self._reset_navigation_state()

    def _on_robot_transform_updated(self, event: DomainEvent) -> None:
        """Update robot position for distance calculations."""
        position = event.data.get('position')
        if position and len(position) >= 2:
            self._robot_position = (float(position[0]), float(position[1]))

    def _on_navigation_stopped(self, event: DomainEvent) -> None:
        """Handle navigation completion or failure."""
        if not self._navigation_active:
            return

        status = event.data.get('status')
        goal_data = event.data.get('goal', {})
        goal_x = goal_data.get('x')
        goal_y = goal_data.get('y')

        self._navigation_active = False

        # Remove NAVIGATE state if it exists
        current = self._state_machine.current_state
        if current and current.value == RobotState.NAVIGATE:
            self._state_machine.pop_state(RobotState.NAVIGATE, RobotStateSource.NAVIGATION)

        if status == 'SUCCEEDED':
            self._logger.info(f"Successfully navigated to target {self._current_target}")
            self._reset_navigation_state()

        elif status == 'ABORTED':
            self._logger.warning(f"Navigation to {self._current_target} was aborted")
            self._handle_navigation_failure()

        elif status == 'CANCELED':
            self._logger.info("Navigation was canceled")
            self._reset_navigation_state()

    def _start_navigation_to_target(self, target_object: str) -> None:
        """Start navigation to the specified target object."""
        self._logger.info(f"Starting navigation to target: {target_object}")

        # Reset state for new target
        if target_object != self._current_target:
            self._reset_navigation_state()
            self._current_target = target_object

        self._attempt_navigation_to_current_target()

    def _attempt_navigation_to_current_target(self) -> None:
        """Attempt to navigate to the current target using the persistent map."""
        if not self._current_target:
            return

        if self._navigation_active:
            return

        # Check if we should wait before retrying
        if self._retry_count > 0:
            time_since_retry = time.time() - self._last_retry_time
            delay = self._retry_delay * (2 ** (self._retry_count - 1))  # Exponential backoff
            if time_since_retry < delay:
                return

        # Check if we've exceeded max retries
        if self._retry_count >= self._max_retry_attempts:
            self._logger.error(f"Max retry attempts ({self._max_retry_attempts}) reached for target {self._current_target}")
            self._give_up_on_target()
            return

        # Find target in persistent map
        target_locations = self._persistent_map.get(self._current_target, [])
        if not target_locations:
            self._logger.debug(f"Target {self._current_target} not found in persistent map yet")
            return

        # Select best location to navigate to
        best_location = self._select_best_target_location(target_locations)
        if not best_location:
            self._logger.warning(f"No viable location found for target {self._current_target}")
            return

        world_coords = best_location.get('world_coords')
        if not world_coords or len(world_coords) < 2:
            self._logger.warning(f"Invalid world coordinates for target {self._current_target}")
            return

        target_x, target_y = float(world_coords[0]), float(world_coords[1])
        self._navigate_to_coordinates(target_x, target_y)

    def _select_best_target_location(self, locations: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
        """Select the best location to navigate to from available options."""
        valid_locations = []

        for loc in locations:
            world_coords = loc.get('world_coords')
            if not world_coords or len(world_coords) < 2:
                continue
            valid_locations.append(loc)

        if not valid_locations:
            return None

        # Prioritize selected targets
        selected_targets = [loc for loc in valid_locations if loc.get('is_selected_target')]
        if selected_targets:
            return selected_targets[0]

        # If no robot position, return most recently seen
        if not self._robot_position:
            return max(valid_locations, key=lambda loc: loc.get('last_seen', 0))

        # Return closest location to robot
        def distance_to_robot(loc):
            coords = loc.get('world_coords')
            if not coords or len(coords) < 2:
                return float('inf')
            dx = float(coords[0]) - self._robot_position[0]
            dy = float(coords[1]) - self._robot_position[1]
            return math.sqrt(dx * dx + dy * dy)

        return min(valid_locations, key=distance_to_robot)

    def _navigate_to_coordinates(self, x: float, y: float) -> None:
        """Send a navigation goal to the specified coordinates."""
        yaw = self._compute_target_yaw(x, y)

        self._logger.info(f"Navigating to target {self._current_target} at ({x:.2f}, {y:.2f})")

        # Update state
        self._navigation_active = True
        self._current_goal_coords = (x, y)
        self._last_retry_time = time.time()

        # Remove any existing OBJECT_FOUND state to prevent conflicts
        current = self._state_machine.current_state
        if current and current.value == RobotState.OBJECT_FOUND and current.source == RobotStateSource.CAMERA:
            self._state_machine.pop_state(RobotState.OBJECT_FOUND, RobotStateSource.CAMERA)

        # Push NAVIGATE state
        self._state_machine.push_state(
            RobotState.NAVIGATE,
            RobotStateSource.NAVIGATION,
            {
                'goal': {'x': x, 'y': y, 'yaw': yaw},
                'status': 'pending',
                'target_object': self._current_target,
                'attempt': self._retry_count + 1
            }
        )

    def _compute_target_yaw(self, target_x: float, target_y: float) -> float:
        """Compute the yaw angle to face the target."""
        if not self._robot_position:
            return 0.0

        dx = target_x - self._robot_position[0]
        dy = target_y - self._robot_position[1]

        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return 0.0

        return math.atan2(dy, dx)

    def _handle_navigation_failure(self) -> None:
        """Handle navigation failure and setup retry."""
        self._retry_count += 1

        if self._retry_count < self._max_retry_attempts:
            delay = self._retry_delay * (2 ** (self._retry_count - 1))
            self._logger.info(f"Navigation failed, will retry in {delay:.1f}s (attempt {self._retry_count + 1}/{self._max_retry_attempts})")
        else:
            self._give_up_on_target()

    def _give_up_on_target(self) -> None:
        """Give up on the current target after max retries."""
        self._logger.error(f"Giving up on target {self._current_target} after {self._max_retry_attempts} attempts")

        # Pop the EXPLORE state to indicate we can't reach this target
        explore_state = self._state_machine.find_state(RobotState.EXPLORE)
        if explore_state and explore_state.source == RobotStateSource.USER:
            self._state_machine.pop_state(RobotState.EXPLORE, RobotStateSource.USER)

        self._reset_navigation_state()

    def _cancel_navigation(self, reason: str) -> None:
        """Cancel any active navigation."""
        if self._navigation_active:
            self._logger.info(f"Canceling navigation: {reason}")
            self._navigation_service.cancel_navigation()
            self._navigation_active = False

        # Remove NAVIGATE state if it exists
        current = self._state_machine.current_state
        if current and current.value == RobotState.NAVIGATE:
            self._state_machine.pop_state(RobotState.NAVIGATE, RobotStateSource.NAVIGATION)

    def _reset_navigation_state(self) -> None:
        """Reset navigation state for a new target."""
        self._current_target = None
        self._navigation_active = False
        self._current_goal_coords = None
        self._retry_count = 0
        self._last_retry_time = 0.0


    def clear_targets(self) -> None:
        """Clear all navigation state and cancel any active navigation."""
        self._logger.info("Clearing all navigation targets")
        self._logger.info("🚫 NAV2 ABORT SOURCE: navigation_controller.clear_targets() - Navigation controller clearing targets")
        self._cancel_navigation("targets cleared")
        self._reset_navigation_state()

    @property
    def current_target(self) -> Optional[str]:
        """Get the current target object name."""
        return self._current_target

    @property
    def is_navigating(self) -> bool:
        """Check if navigation is currently active."""
        return self._navigation_active