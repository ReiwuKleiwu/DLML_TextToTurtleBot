import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import math


def create_pose_stamped(navigator, position_x, position_y, orientation_z):
    # Simple quaternion calculation for yaw rotation
    q_x, q_y = 0.0, 0.0
    q_z = math.sin(orientation_z / 2.0)
    q_w = math.cos(orientation_z / 2.0)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = position_x
    pose.pose.position.y = position_y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose


def main():
    rclpy.init()

    navigator = BasicNavigator()

    print("Waiting for Nav2 to become active...")
    navigator.waitUntilNav2Active()
    print("Nav2 is active!")

    # Set initial pose (not needed for SLAM but can help)
    initial_pose = create_pose_stamped(navigator, 0.0, 0.0, 0.0)
    navigator.setInitialPose(initial_pose)

    # Navigate to goal
    goal_pose = create_pose_stamped(navigator, 1.0, 1.0, 0.0)

    print("Navigating to goal...")
    navigator.goToPose(goal_pose)

    # Wait for completion
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        print(f"Navigation in progress... {i}")
        if i > 60:
            print("Navigation timeout!")
            navigator.cancelTask()
            break
        rclpy.spin_once(navigator, timeout_sec=1.0)

    result = navigator.getResult()
    if result:
        print(f"Navigation completed with result: {result}")
    else:
        print("Navigation failed or was cancelled")

    rclpy.shutdown()


if __name__ == '__main__':
    main()