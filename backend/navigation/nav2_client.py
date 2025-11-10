from __future__ import annotations

import threading
from typing import Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future

from events.event_bus import EventBus
from events.interfaces.events import DomainEvent, EventType


class Nav2Client:
    """Thin wrapper around the Nav2 NavigateToPose action client."""

    def __init__(self, node: Node) -> None:
        self._node = node
        self._event_bus = EventBus()
        self._action_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self._current_goal: Optional[PoseStamped] = None
        self._current_future: Optional[Future] = None
        self._goal_handle = None
        self._lock = threading.Lock()
        self._event_bus.subscribe(EventType.NAVIGATION_CANCEL_REQUEST, self._on_cancel_request)

    def send_goal(self, pose: PoseStamped) -> bool:
        with self._lock:
            if not self._action_client.wait_for_server(timeout_sec=1.0):
                self._node.get_logger().warn("Nav2 action server is not ready")
                return False

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose
            self._current_goal = pose
            future = self._action_client.send_goal_async(goal_msg, feedback_callback=self._on_feedback)
            future.add_done_callback(self._goal_response_callback)
            self._current_future = future
            self._goal_handle = None

            self._event_bus.publish(
                DomainEvent(EventType.NAVIGATION_GOAL_SENT, pose)
            )
            return True

    def cancel_goal(self) -> None:
        with self._lock:
            if self._goal_handle is None:
                return

            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_callback)

    def _goal_response_callback(self, future: Future) -> None:
        goal_handle = future.result()
        if goal_handle is None:
            self._node.get_logger().warn("Nav2 goal rejected")
            self._event_bus.publish(DomainEvent(EventType.NAVIGATION_GOAL_REJECTED, self._current_goal))
            return

        if not goal_handle.accepted:
            self._node.get_logger().warn("Nav2 goal not accepted")
            self._event_bus.publish(DomainEvent(EventType.NAVIGATION_GOAL_REJECTED, self._current_goal))
            return

        self._node.get_logger().info("Nav2 goal accepted")
        self._event_bus.publish(DomainEvent(EventType.NAVIGATION_GOAL_ACCEPTED, self._current_goal))

        with self._lock:
            self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future: Future) -> None:
        response = future.result()

        if response is None:
            self._node.get_logger().warn("Nav2 result unavailable")
            self._event_bus.publish(DomainEvent(EventType.NAVIGATION_GOAL_ABORTED, self._current_goal))
            return

        status = response.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._node.get_logger().info("Nav2 goal succeeded")
            self._event_bus.publish(DomainEvent(EventType.NAVIGATION_GOAL_SUCCEEDED, self._current_goal))
        elif status == GoalStatus.STATUS_CANCELED:
            self._node.get_logger().info("Nav2 goal cancelled")
            self._event_bus.publish(DomainEvent(EventType.NAVIGATION_GOAL_CANCELLED, self._current_goal))
        else:
            self._node.get_logger().warn(f"Nav2 goal failed with status {status}")
            self._event_bus.publish(DomainEvent(EventType.NAVIGATION_GOAL_ABORTED, self._current_goal))

        with self._lock:
            self._current_goal = None
            self._current_future = None
            self._goal_handle = None

    def _cancel_callback(self, future: Future) -> None:
        self._node.get_logger().info("Nav2 goal cancel request sent")
        self._event_bus.publish(DomainEvent(EventType.NAVIGATION_GOAL_CANCELLED, self._current_goal))
        with self._lock:
            self._current_goal = None
            self._current_future = None
            self._goal_handle = None

    def _on_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self._event_bus.publish(DomainEvent(EventType.NAVIGATION_FEEDBACK, feedback))

    def _on_cancel_request(self, event: DomainEvent) -> None:
        self.cancel_goal()
