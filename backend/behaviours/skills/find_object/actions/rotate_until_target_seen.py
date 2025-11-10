import random

import py_trees
from py_trees.common import Status

from utils.twist_wrapper import TwistWrapper


class RotateUntilTargetSeen(py_trees.behaviour.Behaviour):
    """Continuously rotates the robot to search for the target."""

    def __init__(self, name: str, twist: TwistWrapper, publisher) -> None:
        super().__init__(name)
        self._twist = twist
        self._publisher = publisher
        self._angular_speed = 0.4
        self._direction = 1

    def initialise(self) -> None:
        self._direction = random.choice([-1, 1])
        self._twist.reset()
        self._twist.angular.z = self._angular_speed * self._direction
        self._publisher.publish(self._twist.get_message())

    def update(self) -> Status:
        self._twist.angular.z = self._angular_speed * self._direction
        self._publisher.publish(self._twist.get_message())
        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self._twist.reset()
        self._publisher.publish(self._twist.get_message())
