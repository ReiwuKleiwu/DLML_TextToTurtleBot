import random

from classes.controllers.StateMachine import StateMachine, TurtleBotState, TurtleBotStateSource
from classes.perception.ObjectPositionEstimator import ObjectPositionEstimator

class LIDARHandler:
    def __init__(self, state_machine: StateMachine, position_estimator: ObjectPositionEstimator = None):
        self.state_machine = state_machine
        self.position_estimator = position_estimator

    def handle(self, msg):
        # Update position estimator with latest LIDAR data if available
        if self.position_estimator is not None:
            self.position_estimator.update_lidar_data(msg)
        
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        index_front = round((-1.5708 - angle_min) / angle_increment)

        index_start = max(0, index_front - 50)
        index_end = min(len(msg.ranges) - 1, index_front + 50)
        front_ranges = msg.ranges[index_start:index_end + 1]
        front_distance = min([x for x in front_ranges if x != float('inf')])

        if front_distance < 0.5:
            print("[LIDAR]: Detected obstacle closer than 0.5m")
            direction = 1
            self.state_machine.push_state(TurtleBotState.AVOID_OBSTACLE, TurtleBotStateSource.LIDAR, data={"direction": direction})
        else:
            self.state_machine.pop_state(TurtleBotState.AVOID_OBSTACLE, TurtleBotStateSource.LIDAR)