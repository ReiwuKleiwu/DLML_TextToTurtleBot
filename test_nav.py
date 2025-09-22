import rclpy
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


def main(args=None):
    rclpy.init(args=args)

    navigator = TurtleBot4Navigator()

    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    print("Waiting for Nav2 to become active...")
    navigator.waitUntilNav2Active()
    print("Nav2 is active!")

    goal_pose = navigator.getPoseStamped([1.0, 1.0], TurtleBot4Directions.EAST)

    # Undock
    print("Undocking...")
    navigator.undock()

    # Go to each goal pose
    print("Navigating to goal...")
    navigator.startToPose(goal_pose)

    rclpy.shutdown()


if __name__ == '__main__':
    main()