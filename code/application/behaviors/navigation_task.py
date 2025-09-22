"""Behavior to hold navigation goals triggered by the control API."""
from __future__ import annotations

from code.application.utils.twist_wrapper import TwistWrapper
from code.core.interfaces.motion import MotionCommandPublisher
from code.core.state import RobotState, RobotStateSource
from code.core.state_machine import RobotStateMachine
from code.infrastructure.ros.navigation.slam_navigation_service import SLAMNavigationService


class NavigationTask:
    """No-op executor that keeps the control loop engaged while Nav2 runs."""

    def __init__(
        self,
        state_machine: RobotStateMachine,
        navigation_service: SLAMNavigationService,
        twist: TwistWrapper,
        publisher: MotionCommandPublisher,
    ) -> None:
        self._state_machine = state_machine
        self._navigation_service = navigation_service
        self._twist = twist
        self._publisher = publisher

    def execute(self) -> None:
        """Let Nav2 drive; ensure the base velocity command stays neutral."""
        current = self._state_machine.current_state
        if not current or current.value != RobotState.NAVIGATE:
            return

        data = current.data or {}
        goal = data.get('goal')
        if not goal:
            return

        status = data.get('status')
        if status != 'active':
            success = self._navigation_service.navigate_to_pose(
                float(goal.get('x', 0.0)),
                float(goal.get('y', 0.0)),
                float(goal.get('yaw', 0.0)),
            )
            if success:
                data['status'] = 'active'
                current.set_data(data)
            else:
                # Note: This doesn't cancel nav2, but prevents retry by popping the NAVIGATE state
                print("⚠️ NAV2 ABORT SOURCE: navigation_task.execute() - Failed to send goal to Nav2, popping NAVIGATE state")
                self._state_machine.pop_state(RobotState.NAVIGATE, RobotStateSource.USER)
            return

        self._twist.reset()
        self._publisher.publish(self._twist.get_message())
