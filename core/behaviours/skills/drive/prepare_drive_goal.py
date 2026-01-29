import py_trees
from py_trees.common import Status

from shared.blackboard.blackboard import Blackboard
from shared.blackboard.interfaces.blackboard_data_keys import BlackboardDataKey
from core.commands.user_command import UserCommand
from shared.events.event_bus import EventBus
from shared.events.interfaces.events import DomainEvent, EventType


class PrepareDriveGoal(py_trees.behaviour.Behaviour):
    """Capture the starting pose and configure the drive goal on the blackboard."""

    def __init__(self, name: str, command: UserCommand) -> None:
        super().__init__(name)
        self._blackboard: Blackboard = Blackboard()
        self._prepared = False
        self._command = command

    def initialise(self) -> None:
        # Reset prepared flag when behaviour is entered
        self._prepared = False

    def update(self) -> Status:
        if self._prepared:
            return Status.SUCCESS

        position = self._blackboard.get(BlackboardDataKey.ROBOT_POSITION)
        if position is None:
            self.logger.debug("Waiting for robot position before starting drive goal")
            return Status.RUNNING

        parameters = self._command.parameters
        distance = float(parameters.get("distance_m", 0.0))
        if distance <= 0.0:
            self.logger.error("Drive command missing positive distance")
            return Status.FAILURE

        direction = parameters.get("direction")
        if direction is None:
            self.logger.error("Drive command is missing direction")
            return Status.FAILURE

        direction_str = str(parameters.get("direction", "forward")).lower()
        direction_sign = -1 if direction_str in ("backward", "backwards", "reverse", "back") else 1

        start_pose = {"x": position.x, "y": position.y}
        event_bus = EventBus()
        event_bus.publish(
            DomainEvent(
                EventType.DRIVE_GOAL_SET,
                {
                    "target_distance": abs(distance),
                    "start_pose": start_pose,
                    "direction_sign": direction_sign,
                },
            )
        )
        self._prepared = True
        return Status.SUCCESS
