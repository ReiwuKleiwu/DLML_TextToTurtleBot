from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
import math
from typing import Optional


class SLAMNavigationService:
    def __init__(self, node: Node):
        self.node = node

        # Nav2 action client
        self.nav_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

        # Navigation state
        self.current_goal_handle = None
        self.is_active = False

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

        # Send goal
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)

        self.is_active = True
        return True

    def cancel_navigation(self):
        """Cancel current navigation"""
        if self.current_goal_handle and self.is_active:
            self.node.get_logger().info('Canceling navigation goal')
            self.current_goal_handle.cancel_goal_async()

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
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.node.get_logger().error('Navigation goal rejected')
            self.is_active = False
            return

        self.current_goal_handle = goal_handle
        self.node.get_logger().info('Navigation goal accepted')

        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        """Handle navigation result"""
        result = future.result()
        self.is_active = False
        self.current_goal_handle = None

        if result.status == 4:  # SUCCEEDED
            self.node.get_logger().info('Nav2 navigation completed successfully')
        else:
            self.node.get_logger().warn(f'Navigation failed with status: {result.status}')