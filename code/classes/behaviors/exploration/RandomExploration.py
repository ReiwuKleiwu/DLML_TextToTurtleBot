from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Publisher

from classes.behaviors.exploration.ExplorationStrategy import ExplorationStrategy
from classes.utils.TwistWrapper import TwistWrapper

class RandomExploration(ExplorationStrategy):
    def __init__(self, twist: TwistWrapper, cmd_publisher: Publisher):
        self.twist = twist
        self.cmd_publisher = cmd_publisher

    def execute(self):
        self.twist.linear.x = 0.2
        self.twist.angular.z = 0.0
        self.cmd_publisher.publish(self.twist.get_message())
