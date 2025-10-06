from __future__ import annotations

import threading
from functools import partial
from typing import Optional

from action_msgs.msg import GoalStatus
from irobot_create_msgs.action import Dock, Undock
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future

from events.event_bus import EventBus
from events.interfaces.events import DomainEvent, EventType


class DockingClient:
    """Thin wrapper around the TurtleBot4 dock/undock action interfaces."""

    def __init__(self, node: Node) -> None:
        self._node = node
        self._event_bus = EventBus()
        self._dock_client = ActionClient(node, Dock, "dock")
        self._undock_client = ActionClient(node, Undock, "undock")
        self._lock = threading.Lock()
        self._goal_handle = None
        self._current_future: Optional[Future] = None
        self._current_goal_type: Optional[str] = None
        self._event_map = {
            "dock": {
                "sent": EventType.DOCK_GOAL_SENT,
                "accepted": EventType.DOCK_GOAL_ACCEPTED,
                "rejected": EventType.DOCK_GOAL_REJECTED,
                "succeeded": EventType.DOCK_GOAL_SUCCEEDED,
                "aborted": EventType.DOCK_GOAL_ABORTED,
                "cancelled": EventType.DOCK_GOAL_CANCELLED,
                "feedback": EventType.DOCK_FEEDBACK,
            },
            "undock": {
                "sent": EventType.UNDOCK_GOAL_SENT,
                "accepted": EventType.UNDOCK_GOAL_ACCEPTED,
                "rejected": EventType.UNDOCK_GOAL_REJECTED,
                "succeeded": EventType.UNDOCK_GOAL_SUCCEEDED,
                "aborted": EventType.UNDOCK_GOAL_ABORTED,
                "cancelled": EventType.UNDOCK_GOAL_CANCELLED,
                "feedback": EventType.UNDOCK_FEEDBACK,
            },
        }

    def send_dock_goal(self) -> bool:
        goal = Dock.Goal()
        return self._send_goal("dock", self._dock_client, goal)

    def send_undock_goal(self) -> bool:
        goal = Undock.Goal()
        return self._send_goal("undock", self._undock_client, goal)

    def cancel_goal(self) -> None:
        with self._lock:
            if self._goal_handle is None or self._current_goal_type is None:
                return
            goal_type = self._current_goal_type
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(partial(self._cancel_callback, goal_type))

    def _send_goal(self, goal_type: str, client: ActionClient, goal) -> bool:
        with self._lock:
            if self._goal_handle is not None:
                self._node.get_logger().warn("Docking goal already active")
                return False

            if not client.wait_for_server(timeout_sec=1.0):
                self._node.get_logger().warn(
                    f"{goal_type.capitalize()} action server is not ready"
                )
                return False

            future = client.send_goal_async(
                goal,
                feedback_callback=partial(self._on_feedback, goal_type),
            )
            future.add_done_callback(partial(self._goal_response_callback, goal_type))

            self._current_goal_type = goal_type
            self._current_future = future
            self._goal_handle = None

        self._publish_event(goal_type, "sent", goal)
        return True

    def _goal_response_callback(self, goal_type: str, future: Future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._node.get_logger().warn(
                f"{goal_type.capitalize()} goal was rejected"
            )
            self._publish_event(goal_type, "rejected", None)
            with self._lock:
                self._current_goal_type = None
                self._current_future = None
                self._goal_handle = None
            return

        self._node.get_logger().info(f"{goal_type.capitalize()} goal accepted")
        self._publish_event(goal_type, "accepted", None)
        with self._lock:
            self._goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(partial(self._result_callback, goal_type))

    def _result_callback(self, goal_type: str, future: Future) -> None:
        response = future.result()
        result = getattr(response, "result", None)
        status = getattr(response, "status", GoalStatus.STATUS_UNKNOWN)

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._node.get_logger().info(f"{goal_type.capitalize()} goal succeeded")
            self._publish_event(goal_type, "succeeded", result)
        elif status == GoalStatus.STATUS_CANCELED:
            self._node.get_logger().info(f"{goal_type.capitalize()} goal cancelled")
            self._publish_event(goal_type, "cancelled", result)
        else:
            self._node.get_logger().warn(
                f"{goal_type.capitalize()} goal failed with status {status}"
            )
            self._publish_event(goal_type, "aborted", result)

        with self._lock:
            self._current_goal_type = None
            self._current_future = None
            self._goal_handle = None

    def _cancel_callback(self, goal_type: str, future: Future) -> None:
        self._node.get_logger().info(f"{goal_type.capitalize()} goal cancel request sent")
        self._publish_event(goal_type, "cancelled", None)
        with self._lock:
            self._current_goal_type = None
            self._current_future = None
            self._goal_handle = None

    def _on_feedback(self, goal_type: str, feedback_msg) -> None:
        feedback = getattr(feedback_msg, "feedback", feedback_msg)
        self._publish_event(goal_type, "feedback", feedback)

    def _publish_event(self, goal_type: str, phase: str, data) -> None:
        event_type = self._event_map[goal_type][phase]
        self._event_bus.publish(DomainEvent(event_type, data))
