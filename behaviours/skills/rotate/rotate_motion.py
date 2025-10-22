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


class RotateMotion(py_trees.behaviour.Behaviour):
    """Publish angular velocity commands until the requested rotation is achieved."""

    def __init__(self, name: str, command: UserCommand, angular_speed: float = 0.08, tolerance_deg: float = 1.0) -> None:
        super().__init__(name)
        self._blackboard: Blackboard = Blackboard()
        self._twist: Optional[TwistWrapper] = None
        self._publisher = None
        self._command = command
        self._angular_speed = abs(angular_speed)
        self._tolerance = math.radians(max(tolerance_deg, 0.0))
        self._last_yaw: Optional[float] = None
        self._travelled: float = 0.0

    def setup(self, twist: TwistWrapper, publisher) -> None:
        self._twist = twist
        self._publisher = publisher

    def initialise(self) -> None:
        self._halt_motion()
        self._last_yaw = None
        self._travelled = 0.0

    def update(self) -> Status:
        if any(value is None for value in (self._twist, self._publisher)):
            return Status.FAILURE

        target_angle = self._blackboard.get(BlackboardDataKey.ROTATE_TARGET_ANGLE)
        start_yaw = self._blackboard.get(BlackboardDataKey.ROTATE_START_YAW)
        direction_sign = self._blackboard.get(BlackboardDataKey.ROTATE_DIRECTION_SIGN)

        if target_angle is None or start_yaw is None or direction_sign is None:
            self.logger.error("Rotate goal not configured on blackboard")
            return Status.FAILURE

        orientation = self._blackboard.get(BlackboardDataKey.ROBOT_ORIENTATION)
        if orientation is None:
            self.logger.debug("Awaiting orientation updates for rotate progress")
            self._halt_motion()
            return Status.RUNNING

        current_yaw = self._quaternion_to_yaw(orientation)
        if self._last_yaw is None:
            self._last_yaw = start_yaw

        delta = self._normalize_angle(current_yaw - self._last_yaw)
        if direction_sign * delta < 0.0:
            delta = 0.0

        self._travelled += delta
        self._last_yaw = current_yaw

        progress = direction_sign * self._travelled
        if progress < 0.0:
            progress = 0.0

        EventBus().publish(
            DomainEvent(EventType.ROTATE_PROGRESS_UPDATED, min(progress, target_angle))
        )

        if self._goal_reached(progress, target_angle):
            self.logger.info(
                f"Rotate goal reached (target: {math.degrees(target_angle):.1f} deg, travelled: {math.degrees(progress):.1f} deg)"
            )
            self._halt_motion()
            return Status.SUCCESS

        commanded_speed = self._angular_speed * float(direction_sign)
        self._twist.reset()
        self._twist.angular.z = commanded_speed
        self._publisher.publish(self._twist.get_message())
        return Status.RUNNING

    def terminate(self, _: Status) -> None:
        self._halt_motion()

    def _halt_motion(self) -> None:
        if self._twist is None or self._publisher is None:
            return
        self._twist.reset()
        self._publisher.publish(self._twist.get_message())

    @staticmethod
    def _quaternion_to_yaw(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _goal_reached(self, progress: float, target: float) -> bool:
        threshold = max(target - self._tolerance, 0.0)
        return progress >= threshold
