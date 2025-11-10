from py_trees.behaviour import Behaviour
from py_trees.common import Status
from blackboard.blackboard import Blackboard, BlackboardDataKey

class CheckLidar(Behaviour):
    def __init__(self, name: str) -> None:
        super(CheckLidar, self).__init__(name)
        
    def setup(self) -> None:
        self._blackboard = Blackboard()

    def update(self) -> Status:
        lidar_obstacle_present = self._blackboard.get(BlackboardDataKey.LIDAR_OBSTACLE_PRESENT)

        if lidar_obstacle_present:
            return Status.FAILURE
        else:
            return Status.SUCCESS
