import math

import py_trees
from py_trees.common import Status

from shared.blackboard.blackboard import Blackboard
from shared.blackboard.interfaces.blackboard_data_keys import BlackboardDataKey
from backend.commands.user_command import UserCommand
from shared.events.event_bus import EventBus
from shared.events.interfaces.events import DomainEvent, EventType


class PrepareRotateGoal(py_trees.behaviour.Behaviour):
    """Capture the starting orientation and configure the rotate goal on the blackboard."""

    def __init__(self, name: str, command: UserCommand) -> None:
        super().__init__(name)
        self._blackboard: Blackboard = Blackboard()
        self._command = command
        self._prepared = False

    def initialise(self) -> None:
        self._prepared = False

    def update(self) -> Status:
        if self._prepared:
            return Status.SUCCESS

        orientation = self._blackboard.get(BlackboardDataKey.ROBOT_ORIENTATION)
        if orientation is None:
            self.logger.debug("Waiting for robot orientation before starting rotate goal")
            return Status.RUNNING

        yaw = self._quaternion_to_yaw(orientation)

        parameters = self._command.parameters
        angle_deg = float(parameters.get("angle_deg", 0.0))
        if angle_deg <= 0.0:
            self.logger.warn("Rotate command missing positive angle")
            return Status.FAILURE

        angle_rad = math.radians(angle_deg)

        direction = str(parameters.get("direction", "left")).lower()
        direction_sign = -1 if direction in ("right", "clockwise", "cw") else 1

        EventBus().publish(
            DomainEvent(
                EventType.ROTATE_GOAL_SET,
                {
                    "target_angle": abs(angle_rad),
                    "start_yaw": yaw,
                    "direction_sign": direction_sign,
                },
            )
        )
        self._prepared = True
        return Status.SUCCESS

    @staticmethod
    def _quaternion_to_yaw(q) -> float:
        # geometry_msgs.msg.Quaternion with attributes x, y, z, w
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
