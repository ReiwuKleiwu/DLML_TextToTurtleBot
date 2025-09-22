"""Exploration behaviours for the robot."""
from __future__ import annotations

from code.application.utils.twist_wrapper import TwistWrapper
from code.core.interfaces.motion import MotionCommandPublisher


class RandomExploration:
    """Simple behaviour that drives the robot forward."""

    def __init__(self, twist: TwistWrapper, publisher: MotionCommandPublisher) -> None:
        self._twist = twist
        self._publisher = publisher

    def execute(self) -> None:
        self._twist.linear.x = 0.2
        self._twist.angular.z = 0.0
        self._publisher.publish(self._twist.get_message())
