import math
from typing import Dict, Optional, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from classes.events import EventQueue, EventType


class SLAMNavigationService:
    def __init__(self, node: Node):
        self.node = node

        # Nav2 action client
        self.nav_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

        # Navigation state
        self.current_goal_handle = None
        self.is_active = False
        self._last_goal_pose: Optional[Tuple[float, float, float]] = None

        # Event system for broadcasting navigation lifecycle
        self.event_queue = EventQueue()

        self.node.get_logger().info('SLAM Navigation Service initialized')

    def navigate_to_pose(self, x: float, y: float, yaw: float = 0.0) -> bool:
        """Navigate to coordinates using Nav2"""
        if self.is_active:
            self.node.get_logger().info('Canceling previous navigation goal')
            self.cancel_navigation()

        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error('NavigateToPose action server not available')
            return False

        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._create_pose(x, y, yaw)

        self.node.get_logger().info(f'Navigating to: x={x:.2f}, y={y:.2f}')
        self._last_goal_pose = (x, y, yaw)

        # Send goal
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)

        self.is_active = True
        return True

    def cancel_navigation(self):
        """Cancel current navigation"""
        if self.current_goal_handle:
            self.node.get_logger().info('Canceling navigation goal')
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_done_callback)
        else:
            self.node.get_logger().debug('Cancel requested but no active navigation goal handle')

        self.is_active = False

    def is_navigation_active(self) -> bool:
        """Check if navigation is active"""
        return self.is_active

    def _create_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        """Create a PoseStamped message"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.node.get_clock().now().to_msg()

        # Position
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # Orientation (yaw to quaternion)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        return pose

    def _goal_response_callback(self, future):
        """Handle goal response"""
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.node.get_logger().error(f'Failed to send navigation goal: {exc}')
            self.is_active = False
            self._publish_navigation_stopped('ERROR', None)
            self._last_goal_pose = None
            return

        if not goal_handle.accepted:
            self.node.get_logger().error('Navigation goal rejected')
            self.is_active = False
            self._publish_navigation_stopped('REJECTED', GoalStatus.STATUS_UNKNOWN)
            self._last_goal_pose = None
            return

        self.current_goal_handle = goal_handle
        self.node.get_logger().info('Navigation goal accepted')
        self._publish_navigation_started()

        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        """Handle navigation result"""
        status_code: Optional[int] = None
        status_text = 'FAILED'

        try:
            result = future.result()
            status_code = result.status

            if status_code == GoalStatus.STATUS_SUCCEEDED:
                status_text = 'SUCCEEDED'
                self.node.get_logger().info('Nav2 navigation completed successfully')
            elif status_code == GoalStatus.STATUS_CANCELED:
                status_text = 'CANCELED'
                self.node.get_logger().info('Nav2 navigation canceled')
            elif status_code == GoalStatus.STATUS_ABORTED:
                status_text = 'ABORTED'
                self.node.get_logger().warn('Nav2 navigation aborted')
            else:
                status_text = 'FAILED'
                self.node.get_logger().warn(f'Navigation failed with status: {status_code}')
        except Exception as exc:
            status_text = 'ERROR'
            self.node.get_logger().error(f'Navigation result retrieval failed: {exc}')
        finally:
            self.is_active = False
            self.current_goal_handle = None
            self._publish_navigation_stopped(status_text, status_code)
            self._last_goal_pose = None

    def _cancel_done_callback(self, future) -> None:
        try:
            future.result()
        except Exception as exc:
            self.node.get_logger().error(f'Error canceling navigation goal: {exc}')
        finally:
            self.current_goal_handle = None

    def _publish_navigation_started(self) -> None:
        goal_data = self._goal_to_dict(self._last_goal_pose)
        self.event_queue.publish_event(
            EventType.NAVIGATION_STARTED,
            source='SLAMNavigationService',
            data={'goal': goal_data}
        )

    def _publish_navigation_stopped(self, status_text: str, status_code: Optional[int]) -> None:
        goal_data = self._goal_to_dict(self._last_goal_pose)
        self.event_queue.publish_event(
            EventType.NAVIGATION_STOPPED,
            source='SLAMNavigationService',
            data={
                'status': status_text,
                'status_code': status_code,
                'goal': goal_data
            }
        )

    def _goal_to_dict(self, pose: Optional[Tuple[float, float, float]]) -> Dict[str, float]:
        if not pose:
            return {}
        x, y, yaw = pose
        return {'x': x, 'y': y, 'yaw': yaw}
