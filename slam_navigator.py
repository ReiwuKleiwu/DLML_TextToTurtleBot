import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math


class SLAMNavigator(Node):
    def __init__(self):
        super().__init__('slam_navigator')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.initial_pose_pub = self.create_publisher(PoseStamped, 'initialpose', 10)

    def create_pose_stamped(self, position_x, position_y, orientation_z=0.0):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = position_x
        pose.pose.position.y = position_y
        pose.pose.position.z = 0.0
        # Simple quaternion for yaw
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(orientation_z / 2.0)
        pose.pose.orientation.w = math.cos(orientation_z / 2.0)
        return pose

    def set_initial_pose(self, x=0.0, y=0.0, yaw=0.0):
        initial_pose = self.create_pose_stamped(x, y, yaw)
        self.get_logger().info(f'Setting initial pose: x={x}, y={y}, yaw={yaw}')
        self.initial_pose_pub.publish(initial_pose)

    def navigate_to_pose(self, pose):
        # Wait for action server
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for NavigateToPose action server...')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(f'Navigating to: x={pose.pose.position.x}, y={pose.pose.position.y}')

        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        return future


def main():
    rclpy.init()

    navigator = SLAMNavigator()

    # Set initial pose to origin (0,0,0) for SLAM
    navigator.set_initial_pose(0.0, 0.0, 0.0)

    # Wait a moment for initial pose to be processed
    rclpy.spin_once(navigator, timeout_sec=1.0)

    # Create a goal pose
    goal_pose = navigator.create_pose_stamped(1.0, 2.0, 0.0)

    # Send navigation goal
    future = navigator.navigate_to_pose(goal_pose)

    # Spin until goal is sent
    rclpy.spin_until_future_complete(navigator, future)

    goal_handle = future.result()
    if not goal_handle.accepted:
        navigator.get_logger().info('Goal rejected')
        rclpy.shutdown()
        return

    navigator.get_logger().info('Goal accepted, navigating...')

    # Wait for result
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(navigator, result_future)

    result = result_future.result().result
    navigator.get_logger().info(f'Navigation completed with result: {result}')

    rclpy.shutdown()


if __name__ == '__main__':
    main()