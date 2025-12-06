from typing import Optional

import math
import py_trees
from py_trees.common import Status

from core.commands.user_command import UserCommand
from geometry_msgs.msg import PoseStamped, Quaternion
from core.navigation.nav2_client import Nav2Client
from rclpy.node import Node
from shared.events.event_bus import EventBus
from shared.events.interfaces.events import DomainEvent, EventType


class NavigateSkill(py_trees.behaviour.Behaviour):
    """Send a Nav2 goal derived from the command and wait for completion events."""

    def __init__(self, name: str, command: UserCommand, node: Node, nav_client: Nav2Client) -> None:
        super().__init__(name)
        self._command = command
        self._node = node
        self._nav_client = nav_client
        self._event_bus = EventBus()
        self._goal_pose: Optional[PoseStamped] = None
        self._status: Optional[str] = None
        self._event_bus.subscribe(EventType.NAVIGATION_GOAL_SUCCEEDED, self._on_goal_succeeded)
        self._event_bus.subscribe(EventType.NAVIGATION_GOAL_ABORTED, self._on_goal_failed)
        self._event_bus.subscribe(EventType.NAVIGATION_GOAL_CANCELLED, self._on_goal_failed)
        self._event_bus.subscribe(EventType.NAVIGATION_GOAL_REJECTED, self._on_goal_failed)

    def initialise(self) -> None:
        self._status = None
        self._goal_pose = self._build_pose_from_command()
        if self._goal_pose is None:
            self.logger.error("Failed to build navigation goal from command")
            self._status = "failed"
            return
        if not self._nav_client.send_goal(self._goal_pose):
            self._status = "failed"

    def update(self) -> Status:
        if self._status == "failed":
            return Status.FAILURE
        if self._status == "succeeded":
            return Status.SUCCESS
        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        if new_status == Status.INVALID:
            self._nav_client.cancel_goal()

    def _build_pose_from_command(self) -> Optional[PoseStamped]:
        pose_data = self._command.parameters.get("pose")
        if not pose_data:
            return None
        goal = PoseStamped()
        goal.header.stamp = self._node.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = float(pose_data.get("x", 0.0))
        goal.pose.position.y = float(pose_data.get("y", 0.0))
        theta = float(pose_data.get("theta", 0.0))
        goal.pose.orientation = self._yaw_to_quaternion(theta)
        return goal

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        half_yaw = yaw * 0.5
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(half_yaw)
        quat.w = math.cos(half_yaw)
        return quat

    # Nav2 is a bit of a special child, so instead of fetching the Blackboard we directly listen to the event queue
    def _on_goal_succeeded(self, event: DomainEvent) -> None:
        self._status = "succeeded"
        self.logger.info("Navigation goal succeeded")

    def _on_goal_failed(self, event: DomainEvent) -> None:
        self._status = "failed"
        self.logger.info("Navigation goal failed")
