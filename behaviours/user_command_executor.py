from typing import Optional

import py_trees
from py_trees.common import Status

from blackboard.interfaces.blackboard_data_keys import BlackboardDataKey
from blackboard.blackboard import Blackboard
from commands.user_command import CommandType, UserCommand
from behaviours.skills.drive.drive_skill import DriveSkill
from behaviours.skills.rotate.rotate_skill import RotateSkill
from behaviours.skills.navigate.navigate_skill import NavigateSkill
from behaviours.skills.find_object.find_object_skill import FindObjectSkill
from navigation.nav2_client import Nav2Client
from rclpy.node import Node
from events.event_bus import EventBus
from events.interfaces.events import DomainEvent, EventType
from utils.twist_wrapper import TwistWrapper


class UserCommandExecutor(py_trees.behaviour.Behaviour):
    """Dispatch user commands to the appropriate skill subtree."""

    def __init__(self, name: str, nav_client: Nav2Client, node: Node) -> None:
        super().__init__(name)
        self._blackboard: Blackboard = Blackboard()
        self._event_bus: EventBus = EventBus()
        self._twist: Optional[TwistWrapper] = None
        self._publisher = None
        self._active_skill: Optional[py_trees.behaviour.Behaviour] = None
        self._active_command: Optional[UserCommand] = None
        self._nav_client = nav_client
        self._node = node

    def setup(
        self,
        twist: TwistWrapper,
        publisher,
    ) -> None:  # type: ignore[override]
        self._twist = twist
        self._publisher = publisher

    def update(self) -> Status:
        if any(dep is None for dep in (self._twist, self._publisher)):
            self.logger.error("UserCommandExecutor dependencies not initialised")
            return Status.FAILURE

        if self._active_skill is None:
            if not self._build_command_skill():
                return Status.RUNNING

        status = Status.RUNNING
        if self._active_skill is not None:
            self._active_skill.tick_once()
            status = self._active_skill.status

        active_from_blackboard = self._blackboard.get(BlackboardDataKey.ACTIVE_COMMAND)
        if self._active_command is not None and (
            active_from_blackboard is None
            or active_from_blackboard.command_id != self._active_command.command_id
        ):
            # Command was cancelled externally or cleared by safety
            self._handle_command_cancelled()
            return Status.RUNNING

        if status == Status.SUCCESS:
            self._handle_command_success()
            return Status.RUNNING

        if status == Status.FAILURE:
            self._handle_command_failure()
            return Status.FAILURE

        return Status.RUNNING


    def _handle_command_success(self) -> None:
        if self._active_command is not None:
            self._event_bus.publish(DomainEvent(EventType.COMMAND_COMPLETED, self._active_command))
        self._active_skill = None
        self._active_command = None
         
    def _handle_command_failure(self) -> None:
        if self._active_command is not None:
            self._event_bus.publish(DomainEvent(EventType.COMMAND_CANCELLED, self._active_command))
        self._active_skill = None
        self._active_command = None
    
    def _handle_command_cancelled(self) -> None:
        if self._active_command is not None:
            self._event_bus.publish(DomainEvent(EventType.COMMAND_CANCELLED, self._active_command))
        self._active_skill = None
        self._active_command = None

    def _build_command_skill(self) -> bool:
        next_command = self._blackboard.peek_command()
        if next_command is None:
            return False

        skill: Optional[py_trees.behaviour.Behaviour] = None
        command: Optional[UserCommand] = None

        if next_command.command_type == CommandType.DRIVE:
            command = self._blackboard.pop_command()
            if command is not None:
                skill = DriveSkill(f"DriveSkill-{command.command_id[:8]}", command)
                skill.setup(self._twist, self._publisher)
        elif next_command.command_type == CommandType.ROTATE:
            command = self._blackboard.pop_command()
            if command is not None:
                skill = RotateSkill(f"RotateSkill-{command.command_id[:8]}", command)
                skill.setup(self._twist, self._publisher)
        elif next_command.command_type == CommandType.NAVIGATE_TO_POSE:
            command = self._blackboard.pop_command()
            if command is not None:
                skill = NavigateSkill(
                    f"NavigateSkill-{command.command_id[:8]}",
                    command,
                    self._node,
                    self._nav_client,
                )
        elif next_command.command_type == CommandType.FIND_OBJECT:
            command = self._blackboard.pop_command()
            if command is not None:
                skill = FindObjectSkill(
                    f"FindObjectSkill-{command.command_id[:8]}",
                    command,
                    self._node,
                    self._nav_client,
                    self._twist,
                    self._publisher,
                )
        else:
            self.logger.warn(f"Unsupported command type: {next_command.command_type}")
            dropped = self._blackboard.pop_command()
            if dropped is not None:
                self._event_bus.publish(DomainEvent(EventType.COMMAND_CANCELLED, dropped))
            return False

        if skill is None or command is None:
            return False

        self._event_bus.publish(DomainEvent(EventType.COMMAND_STARTED, command))
        self._active_skill = skill
        self._active_command = command
        return True
