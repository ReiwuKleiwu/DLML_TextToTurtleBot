import py_trees
from py_trees.common import Status

from backend.behaviours.skills.find_object.actions.set_target_class import SetTargetClass
from backend.behaviours.skills.find_object.conditions.map_has_target import MapHasTarget
from backend.behaviours.skills.find_object.actions.send_nav_goal_to_target import SendNavGoalToTarget
from backend.behaviours.skills.find_object.conditions.wait_for_nav_or_target import WaitForNavOrTarget
from backend.behaviours.skills.find_object.actions.rotate_until_target_seen import RotateUntilTargetSeen
from backend.behaviours.skills.find_object.actions.align_to_target import AlignToTarget
from backend.behaviours.skills.find_object.actions.approach_target import ApproachTarget
from backend.behaviours.skills.find_object.conditions.target_selected import TargetSelected
from backend.behaviours.skills.find_object.conditions.wait_for_target_reached import WaitForTargetReached
from backend.commands.user_command import UserCommand
from backend.navigation.nav2_client import Nav2Client
from shared.utils.twist_wrapper import TwistWrapper
from rclpy.node import Node
from shared.events.event_bus import EventBus
from shared.events.interfaces.events import DomainEvent, EventType


class FindObjectSkill(py_trees.composites.Sequence):
    """Sequence that sets the target class and then locates the object."""

    def __init__(
        self,
        name: str,
        command: UserCommand,
        node: Node,
        nav_client: Nav2Client,
        twist: TwistWrapper,
        publisher,
    ) -> None:
        super().__init__(name, memory=False)
        target_class = str(command.parameters.get("object_class", "")).lower()
        self._target_class = target_class
        self._event_bus = EventBus()

        set_target = SetTargetClass("SetTargetClass", target_class)
        acquisition = py_trees.composites.Selector("AcquireTarget", memory=False)
        acquisition.add_children([
            self._build_navigation_branch(node, nav_client),
            self._build_exploration_branch(twist, publisher),
        ])

        self.add_children([set_target, acquisition])

    def terminate(self, new_status: Status) -> None:
        super().terminate(new_status)
        if self._target_class:
            self._event_bus.publish(DomainEvent(EventType.TARGET_OBJECT_CLASS_SET, None))

    def _build_navigation_branch(self, node: Node, nav_client: Nav2Client) -> py_trees.behaviour.Behaviour:
        sequence = py_trees.composites.Sequence("MapNavigation", memory=False)
        sequence.add_children([
            MapHasTarget("TargetInMap"),
            SendNavGoalToTarget("SendNavGoal", node, nav_client),
            WaitForNavOrTarget("NavOrDetector"),
        ])
        return sequence

    def _build_exploration_branch(self, twist: TwistWrapper, publisher) -> py_trees.behaviour.Behaviour:
        rotation = RotateUntilTargetSeen("RotateUntilSeen", twist, publisher)
        alignment = AlignToTarget("AlignToTarget", twist, publisher)
        approach = ApproachTarget("ApproachTarget", twist, publisher)
        wait_reached = WaitForTargetReached("WaitForTargetReached")

        acquire_selector = py_trees.composites.Selector("AcquireDetection", memory=False)
        acquire_selector.add_children([
            TargetSelected("TargetAlreadySelected"),
            rotation,
        ])

        sequence = py_trees.composites.Sequence("Exploration", memory=False)
        sequence.add_children([
            acquire_selector,
            alignment,
            approach,
            wait_reached,
        ])
        return sequence
