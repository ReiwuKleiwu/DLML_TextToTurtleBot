from geometry_msgs.msg import Twist
from rclpy.node import Publisher

from classes.behaviors.obstacle_avoidance.ObstacleAvoidanceStrategy import ObstacleAvoidanceStrategy
from classes.controllers.StateMachine import StateMachine

class SimpleObstacleAvoidance(ObstacleAvoidanceStrategy):
    def __init__(self, state_machine: StateMachine, twist: Twist, cmd_publisher: Publisher):
        self.state_machine = state_machine
        self.twist = twist
        self.cmd_publisher = cmd_publisher

    def execute(self):
        self.twist.twist.linear.x = 0.0
        direction = self.state_machine.get_current_state().data.get('direction', 1)
        self.twist.twist.angular.z = 0.4 * direction

        self.cmd_publisher.publish(self.twist)
