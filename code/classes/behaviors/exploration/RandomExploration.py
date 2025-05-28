from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Publisher

from classes.behaviors.exploration.ExplorationStrategy import ExplorationStrategy

class RandomExploration(ExplorationStrategy):
    def __init__(self, twist: Twist | TwistStamped, cmd_publisher: Publisher):
        self.twist = twist
        self.cmd_publisher = cmd_publisher

    def execute(self):
        self.twist.twist.linear.x = 0.2
        self.twist.twist.angular.z = 0.0
        self.cmd_publisher.publish(self.twist)
