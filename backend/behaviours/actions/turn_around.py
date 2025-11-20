from py_trees.behaviour import Behaviour
from py_trees.common import Status

from shared.utils.twist_wrapper import TwistWrapper


class TurnAround(Behaviour):
    def __init__(self, name: str) -> None:
        super().__init__(name)

    def setup(self, twist: TwistWrapper, publisher) -> None:
        self.twist = twist
        self.publisher = publisher

    def update(self) -> Status:
        self.twist.reset()
        self.twist.angular.z = 1.0
        self.publisher.publish(self.twist.get_message())
        return Status.RUNNING
