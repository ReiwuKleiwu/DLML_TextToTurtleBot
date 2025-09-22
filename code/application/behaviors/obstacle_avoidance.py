"""Obstacle avoidance behaviour implementations."""
from __future__ import annotations

from code.application.utils.twist_wrapper import TwistWrapper
from code.core.interfaces.motion import MotionCommandPublisher
from code.core.state_machine import RobotStateMachine
from code.core.state import RobotState


class SimpleObstacleAvoidance:
    """Rotate in place to avoid a detected obstacle."""

    def __init__(
        self,
        state_machine: RobotStateMachine,
        twist: TwistWrapper,
        publisher: MotionCommandPublisher,
    ) -> None:
        self._state_machine = state_machine
        self._twist = twist
        self._publisher = publisher

    def execute(self) -> None:
        current = self._state_machine.current_state
        direction = 1
        if current and current.data.get('direction') is not None:
            try:
                direction = int(current.data['direction'])
            except (TypeError, ValueError):
                direction = 1

        self._twist.linear.x = 0.0
        self._twist.angular.z = 0.4 * direction
        self._publisher.publish(self._twist.get_message())
