import rclpy
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from classes.nodes.TextToTurtlebotNode import TextToTurtlebotNode
from classes.controllers.MovementThread import MovementThread

# class GoalPublisher(Node):
#     def __init__(self):
#         super().__init__('goal_publisher')
#         self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
#         timer_period = 1.0  # seconds
#         self.timer = self.create_timer(timer_period, self.publish_goal)
#
#     def publish_goal(self):
#         msg = PoseStamped()
#         msg.header.frame_id = 'map'
#         # msg.header.stamp = self.get_clock().now().to_msg()
#         msg.header.stamp.sec = 0
#         msg.pose.position.x = -7.0
#         msg.pose.position.y = 0.0
#         msg.pose.position.z = 0.0
#         msg.pose.orientation.w = 1.0  # yaw = 0
#         self.publisher.publish(msg)
#         self.get_logger().info('Published goal_pose')
#         self.destroy_timer(self.timer)  # one-shot publish


class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_goal_pose)

    def publish_goal_pose(self):
        msg = PoseStamped()

        print("publishing goal")

        # Set header
        # msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        msg.header.stamp.sec = 0
        msg.header.frame_id = 'map'

        # Set pose
        msg.pose.position.x = 4.0
        msg.pose.position.y = 4.0
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0

        self.pose_publisher.publish(msg)

        print("B")

        # self.destroy_timer(self.timer)  # one-shot publish



def main(args=None):
    rclpy.init(args=args)
    # node = GoalPublisher()
    # rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()
    # return
    text_to_turtlebot_node = TextToTurtlebotNode(namespace="", use_turtlebot_sim=True)
    text_to_turtlebot_node.find_target('chair')

    try:
        navigator = TurtleBot4Navigator()

        if not navigator.getDockedStatus():
            navigator.info('Docking before initialising pose')
            # navigator.dock()

        # initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        # navigator.setInitialPose(initial_pose)

        # navigator.waitUntilNav2Active() # localizer='slam_toolbox'
        # navigator.lifecycleStartup()

        # goal_pose = navigator.getPoseStamped([-13.0, 9.0], TurtleBot4Directions.EAST)

        # navigator.undock()

        # navigator.startToPose(goal_pose)

        movement_thread = MovementThread(2, text_to_turtlebot_node)
        rclpy.spin(text_to_turtlebot_node)
        text_to_turtlebot_node.destroy_node()
        rclpy.shutdown()
    finally:
        movement_thread.end()
    

if __name__ == '__main__':
    main()
