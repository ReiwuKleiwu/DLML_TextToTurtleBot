import math
from typing import Optional
import py_trees
from py_trees.common import Status

from blackboard.blackboard import Blackboard
from blackboard.interfaces.blackboard_data_keys import BlackboardDataKey
from commands.user_command import UserCommand
from utils.twist_wrapper import TwistWrapper
from events.event_bus import EventBus
from events.interfaces.events import DomainEvent, EventType

class DriveMotion(py_trees.behaviour.Behaviour):
    """Publish velocity commands until the requested distance has been travelled."""

    def __init__(self, name: str, command: UserCommand, speed: float = 0.25, tolerance: float = 0.0) -> None:
        super().__init__(name)
        self._blackboard: Blackboard = Blackboard()
        self._twist: Optional[TwistWrapper] = None
        self._publisher = None
        self._command = command
        self._speed = abs(speed)
        self._tolerance = max(tolerance, 0.0)

    def setup(self, twist: TwistWrapper, publisher) -> None:
        self._twist = twist
        self._publisher = publisher

    def initialise(self) -> None:
        # Ensure the robot is stationary at the start of the motion phase
        self._halt_motion()

    def update(self) -> Status:
        if any(value is None for value in (self._twist, self._publisher)):
            return Status.FAILURE

        target_distance = self._blackboard.get(BlackboardDataKey.DRIVE_TARGET_DISTANCE)
        start_pose = self._blackboard.get(BlackboardDataKey.DRIVE_START_POSE)
        direction_sign = self._blackboard.get(BlackboardDataKey.DRIVE_DIRECTION_SIGN, 1)

        if target_distance is None or start_pose is None or direction_sign is None:
            self.logger.warn("Drive goal not configured on blackboard")
            return Status.FAILURE

        current_pose = self._blackboard.get(BlackboardDataKey.ROBOT_POSITION)
        if current_pose is None:
            self.logger.debug("Awaiting current pose updates for drive progress")
            self._halt_motion()
            return Status.RUNNING

        dx = current_pose.x - start_pose["x"]
        dy = current_pose.y - start_pose["y"]
        travelled = math.hypot(dx, dy)

        # Determine signed progress: forward = +, backward = -
        signed_travelled = travelled * float(direction_sign)
        EventBus().publish(
            DomainEvent(EventType.DRIVE_PROGRESS_UPDATED, abs(signed_travelled))
        )

        target_with_sign = float(direction_sign) * float(target_distance)
        distance_remaining = target_with_sign - signed_travelled

        if (direction_sign > 0 and distance_remaining <= self._tolerance) or (
            direction_sign < 0 and distance_remaining >= -self._tolerance
        ):
            self.logger.info(
                f"Drive goal reached (target: {target_distance:.2f} m, travelled: {abs(signed_travelled):.2f} m)"
            )
            self._halt_motion()
            return Status.SUCCESS

        commanded_speed = self._speed * float(direction_sign)
        self._twist.reset()
        self._twist.linear.x = commanded_speed
        self._publisher.publish(self._twist.get_message())
        return Status.RUNNING

    def terminate(self, _: Status) -> None: 
        self._halt_motion()

    def _halt_motion(self) -> None:
        if self._twist is None or self._publisher is None:
            return
        self._twist.linear.x = 0.0
        self._publisher.publish(self._twist.get_message())
        self._twist.reset()
