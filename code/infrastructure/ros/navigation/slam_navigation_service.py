"""Wrapper around Nav2 action client for navigation goals."""
from __future__ import annotations

import math
from typing import Any, Dict, Optional, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from code.core.events import EventType
from code.core.interfaces.event_bus import EventBus


class SLAMNavigationService:
    """High-level interface to the Nav2 NavigateToPose action."""

    def __init__(self, node: Node, event_bus: EventBus) -> None:
        self._node = node
        self._bus = event_bus

        self._nav_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self._current_goal_handle = None
        self._is_active = False
        self._last_goal_pose: Optional[Tuple[float, float, float]] = None
        self._last_status_text: str = 'IDLE'
        self._last_status_code: Optional[int] = None

        self._node.get_logger().info('SLAM Navigation Service initialized')

    def navigate_to_pose(self, x: float, y: float, yaw: float = 0.0) -> bool:
        if self._is_active:
            self._node.get_logger().info('Canceling previous navigation goal')
            self._node.get_logger().info('🚫 NAV2 ABORT SOURCE: slam_navigation_service.navigate_to_pose() - New goal requested while previous goal active')
            self.cancel_navigation()

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self._node.get_logger().error('NavigateToPose action server not available')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._create_pose(x, y, yaw)

        self._node.get_logger().info(f'Navigating to: x={x:.2f}, y={y:.2f}')
        self._last_goal_pose = (x, y, yaw)
        self._last_status_text = 'ACTIVE'
        self._last_status_code = None

        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)

        self._is_active = True
        return True

    def cancel_navigation(self) -> None:
        if self._current_goal_handle:
            self._node.get_logger().info('Canceling navigation goal')
            self._node.get_logger().info('🚫 NAV2 ABORT SOURCE: slam_navigation_service.cancel_navigation() - Direct cancellation request')
            cancel_future = self._current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_done_callback)
            self._last_status_text = 'CANCELED'
            self._last_status_code = GoalStatus.STATUS_CANCELED
        else:
            self._node.get_logger().debug('Cancel requested but no active navigation goal handle')
        self._is_active = False

    def is_navigation_active(self) -> bool:
        return self._is_active

    def _create_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self._node.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _goal_response_callback(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().error(f'Failed to send navigation goal: {exc}')
            self._is_active = False
            self._publish_navigation_stopped('ERROR', None)
            self._last_goal_pose = None
            return

        if not goal_handle.accepted:
            self._node.get_logger().error('Navigation goal rejected')
            self._is_active = False
            self._publish_navigation_stopped('REJECTED', GoalStatus.STATUS_UNKNOWN)
            self._last_goal_pose = None
            return

        self._current_goal_handle = goal_handle
        self._node.get_logger().info('Navigation goal accepted')
        self._publish_navigation_started()

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future) -> None:
        status_code: Optional[int] = None
        status_text = 'FAILED'

        try:
            result = future.result()
            status_code = result.status

            if status_code == GoalStatus.STATUS_SUCCEEDED:
                status_text = 'SUCCEEDED'
                self._node.get_logger().info('Nav2 navigation completed successfully')
            elif status_code == GoalStatus.STATUS_CANCELED:
                status_text = 'CANCELED'
                self._node.get_logger().info('Nav2 navigation canceled')
            elif status_code == GoalStatus.STATUS_ABORTED:
                status_text = 'ABORTED'
                self._node.get_logger().info(f"Result: {result}")
                self._node.get_logger().warning('Nav2 navigation aborted')
                self._node.get_logger().error('🚫 NAV2 ABORT SOURCE: Nav2 action server - Navigation aborted by Nav2 itself (obstacle/unreachable/planner failure)')
            else:
                status_text = 'FAILED'
                self._node.get_logger().warning(f'Navigation failed with status: {status_code}')
        except Exception as exc:  # noqa: BLE001
            status_text = 'ERROR'
            self._node.get_logger().error(f'Navigation result retrieval failed: {exc}')
        finally:
            self._is_active = False
            self._current_goal_handle = None
            self._publish_navigation_stopped(status_text, status_code)
            self._last_goal_pose = None

    def _cancel_done_callback(self, future) -> None:
        try:
            future.result()
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().error(f'Error canceling navigation goal: {exc}')
        finally:
            self._current_goal_handle = None

    def _publish_navigation_started(self) -> None:
        goal_data = self._goal_to_dict(self._last_goal_pose)
        self._bus.publish_event(
            EventType.NAVIGATION_STARTED,
            source='SLAMNavigationService',
            data={'goal': goal_data},
        )
        self._last_status_text = 'ACTIVE'
        self._last_status_code = None

    def _publish_navigation_stopped(self, status_text: str, status_code: Optional[int]) -> None:
        goal_data = self._goal_to_dict(self._last_goal_pose)
        self._bus.publish_event(
            EventType.NAVIGATION_STOPPED,
            source='SLAMNavigationService',
            data={
                'status': status_text,
                'status_code': status_code,
                'goal': goal_data,
            },
        )
        self._last_status_text = status_text
        self._last_status_code = status_code

    def _goal_to_dict(self, pose: Optional[Tuple[float, float, float]]) -> Dict[str, float]:
        if not pose:
            return {}
        x, y, yaw = pose
        return {'x': x, 'y': y, 'yaw': yaw}

    def get_last_goal(self) -> Optional[Tuple[float, float, float]]:
        """Return the most recently requested goal pose."""
        return self._last_goal_pose

    def get_last_status(self) -> Dict[str, Any]:
        """Return the most recent navigation status text and code."""
        return {
            'status': self._last_status_text,
            'status_code': self._last_status_code,
        }
