import ExplorationStrategy

class RandomExploration(ExplorationStrategy):
    def __init__(self, twist, cmd_publisher):
        self.twist = twist
        self.cmd_publisher = cmd_publisher

    def execute(self):
        self.twist.linear.x = 0.2
        self.twist.angular.z = 0.0
        self.cmd_publisher.publish(self.twist)
