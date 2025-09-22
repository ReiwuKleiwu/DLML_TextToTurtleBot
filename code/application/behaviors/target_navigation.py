"""Behaviours for guiding the robot toward detected targets."""
from __future__ import annotations

from code.application.utils.twist_wrapper import TwistWrapper
from code.core.interfaces.motion import MotionCommandPublisher
from code.core.state_machine import RobotStateMachine
from code.core.state import RobotState


class SimpleTargetNavigation:
    """Turn toward the selected target and approach it."""

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
        state = self._state_machine.current_state
        if not state or state.value != RobotState.OBJECT_FOUND:
            return

        data = state.data.get('object_found_data')
        if not data:
            print("[TARGET NAVIGATION - ERROR]: Detected target object is not defined.")
            return

        bbox = data.get('bounding_box_coordinates', {})
        camera = data.get('camera', {})
        try:
            width = bbox['x2'] - bbox['x1']
            center = bbox['x1'] + (width / 2.0)
            image_center = camera['width'] / 2.0
        except KeyError:
            return

        offset = center - image_center
        if abs(offset) >= 25:
            normalized_turn = self._map_to_minus1_to_1(
                offset,
                -(camera['width'] / 2.0),
                camera['width'] / 2.0,
            )
            self._twist.linear.x = 0.0
            self._twist.angular.z = -1.0 * normalized_turn
        else:
            self._twist.linear.x = 0.2
            self._twist.angular.z = 0.0

        self._publisher.publish(self._twist.get_message())

    def _map_to_minus1_to_1(self, value: float, low: float, high: float) -> float:
        return 2.0 * (value - low) / (high - low) - 1.0
